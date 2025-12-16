#!/usr/bin/env python3
import math
import time
import asyncio
import threading
import json
import random
from typing import Optional, List, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Point, Twist, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import String, Float32

from robot_interfaces.msg import Bid, SpawnedPackage
from robot_interfaces.action import ExecuteShift

# -----------------------
# Configuration constants
# -----------------------
BID_DISTANCE_WEIGHT = 1.0       # distance multiplier
POSE_TOLERANCE = 0.2            # meters
STATE_PUBLISH_INTERVAL = 1.0    # s
BID_WAIT_TIME = 1.5             # seconds to wait for other bids

# Collision avoidance constants
COLLISION_RADIUS = 2.0          # meters - minimum distance between robots
COLLISION_SLOW_RADIUS = 3.5     # meters - distance to start slowing down
COLLISION_WAIT_TIME = 0.1       # seconds between collision checks
YIELD_TIMEOUT = 3.0             # seconds before deadlock recovery kicks in
MIN_VELOCITY = 0.3              # minimum velocity when avoiding
RANDOM_BACKOFF_MAX = 0.5        # max random backoff in seconds

# Battery constants
BATTERY_DRAIN_PER_METER = 0.016  # 1.6% per meter
LOW_BATTERY_THRESHOLD = 0.20     # 20% threshold for charging
CHARGING_TIME = 10.0             # seconds to charge
BATTERY_PENALTY_WEIGHT = 2.0     # weight for battery penalty in bids


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        # Parameter for all robot names (for collision avoidance)
        self.declare_parameter('all_robot_names', ['robot1', 'robot2'])
        self.all_robot_names = self.get_parameter('all_robot_names').get_parameter_value().string_array_value

        # State
        self.state = 'idle' # idle, bidding, retrieving, delivering, charging, returning
        self.current_pose: Optional[PoseStamped] = None
        self.current_yaw = 0.0 # Internal yaw for dead reckoning
        self.current_path: Optional[Path] = None
        self._path_waiter = None
        
        # Collision avoidance: track other robots' positions
        self.other_poses: Dict[str, PoseStamped] = {}
        
        # Session state
        self.active_session = False
        self.packages_delivered = 0
        self.expected_packages = 0
        self.current_task = None # {task_id, shelf_pos, shelf_id}
        self.bids_received: Dict[str, Dict] = {} # task_id -> {robot_name -> cost}
        self.home_pose: Optional[Point] = None # Return here after delivery
        
        self.delivery_points: List[Point] = [] # Will be updated from config
        
        # Battery state
        self.battery_level = 1.0  # 100% initial
        self.charging_station_pos: Optional[Point] = None
        self.is_charging = False
        self.waiting_for_charging = False
        self.charging_station_occupied = False
        self.charging_queue: List[str] = []  # Queue of robot names
        
        # Collision avoidance state
        self.yield_start_time: Optional[float] = None  # Track when we started yielding

        # Callback Groups
        self.cb_group = ReentrantCallbackGroup()
        
        # Concurrency Lock
        self._state_lock = threading.Lock()

        # Publishers
        self.state_pub = self.create_publisher(String, f'/{self.robot_name}/state', 10)
        self.bid_pub = self.create_publisher(Bid, '/robot_bids', 10)
        self.goal_pub = self.create_publisher(PoseStamped, f'/{self.robot_name}/goal', 10)
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        
        # Pose Publisher (Dead Reckoning)
        self.pose_pub = self.create_publisher(PoseStamped, f'/{self.robot_name}/pose', 10)
        self.task_completed_pub = self.create_publisher(String, '/task_completed', 10)
        
        # Charging station request publisher
        self.charging_request_pub = self.create_publisher(String, f'/{self.robot_name}/charging_request', 10)

        # Subscribers
        self.create_subscription(SpawnedPackage, '/spawned_package', self.spawn_cb, 10, callback_group=self.cb_group)
        self.create_subscription(Bid, '/robot_bids', self.bid_cb, 10, callback_group=self.cb_group)
        self.create_subscription(Path, f'/{self.robot_name}/path', self.path_cb, 10, callback_group=self.cb_group)
        
        # Subscribe to other robots' poses for collision avoidance
        for other_name in self.all_robot_names:
            if other_name != self.robot_name:
                topic = f'/{other_name}/pose'
                self.create_subscription(
                    PoseStamped, topic,
                    lambda msg, n=other_name: self.other_pose_cb(msg, n),
                    10, callback_group=self.cb_group
                )
                self.get_logger().info(f'[{self.robot_name}] Subscribed to {topic} for collision avoidance')
        
        # Config Subscriber
        self.create_subscription(String, '/simulation/config', self.config_cb, 10, callback_group=self.cb_group)
        
        # Charging station status subscriber
        self.create_subscription(String, '/charging_station/status', self.charging_status_cb, 10, callback_group=self.cb_group)

        # Action Server
        self._action_server = ActionServer(
            self,
            ExecuteShift,
            f'/{self.robot_name}/execute_shift',
            execute_callback=self.execute_session_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        # Timers
        self.create_timer(STATE_PUBLISH_INTERVAL, self.publish_state, callback_group=self.cb_group)

        # Asyncio Loop Thread
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.loop_thread.start()

        self.get_logger().info(f'[{self.robot_name}] RobotNode initialized. Waiting for config...')

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    # -----------------------
    # Config Handling
    # -----------------------
    def config_cb(self, msg: String):
        if self.current_pose is not None:
            pass
        
        try:
            data = json.loads(msg.data)
            
            # Parse delivery points which is a list now
            # Expected: "delivery_points": [{"x": 1.0, "z": 2.0}, ...]
            # Fallback for old "delivery" key for backward compatibility if desired, or just switch.
            # User asked for modification, so let's check for new key.
            
            if 'delivery_points' in data:
                self.delivery_points = []
                for dp in data['delivery_points']:
                    self.delivery_points.append(Point(x=float(dp['x']), y=0.0, z=float(dp['z'])))
                self.get_logger().info(f"[{self.robot_name}] Loaded {len(self.delivery_points)} delivery points.")
            elif 'delivery' in data:
                # Legacy single point support
                d = data['delivery']
                self.delivery_points = [Point(x=float(d['x']), y=0.0, z=float(d['z']))]
                self.get_logger().info(f"[{self.robot_name}] Loaded single delivery point.")

            if self.current_pose is None and 'robots' in data:
                for r in data['robots']:
                    if r['name'] == self.robot_name:
                        # Set initial pose
                        p = PoseStamped()
                        p.header.frame_id = 'map'
                        p.header.stamp = self.get_clock().now().to_msg()
                        p.pose.position.x = float(r['x'])
                        p.pose.position.z = float(r['z'])
                        p.pose.orientation.w = 1.0 # Identity quaternion
                        
                        self.current_pose = p
                        self.current_yaw = 0.0 
                        self.home_pose = Point(x=float(r['x']), y=0.0, z=float(r['z']))
                        
                        # Publish initial pose
                        self.pose_pub.publish(p)
                        self.get_logger().info(f"[{self.robot_name}] Initial pose set: ({r['x']}, {r['z']})")
                        break
            
            # Parse charging station
            if 'charging_station' in data:
                cs = data['charging_station']
                self.charging_station_pos = Point(x=float(cs['x']), y=0.0, z=float(cs['z']))
                self.get_logger().info(f"[{self.robot_name}] Charging station at ({cs['x']}, {cs['z']})")
                        
        except Exception as e:
            self.get_logger().error(f"[{self.robot_name}] Error parsing config: {e}")

    # -----------------------
    # Publishers
    # -----------------------
    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def publish_pose_periodically(self):
        # Republish pose periodically if idle, to keep Unity updated
        if self.current_pose:
             self.current_pose.header.stamp = self.get_clock().now().to_msg()
             self.pose_pub.publish(self.current_pose)

    # -----------------------
    # Callbacks
    # -----------------------
    def path_cb(self, msg: Path):
        self.current_path = msg
        if self._path_waiter and not self._path_waiter.done():
            self.loop.call_soon_threadsafe(self._path_waiter.set_result, msg)

    def bid_cb(self, msg: Bid):
        # Store bids from other robots
        if msg.task_id not in self.bids_received:
            self.bids_received[msg.task_id] = {}
        self.bids_received[msg.task_id][msg.robot_name] = msg.cost
    
    def charging_status_cb(self, msg: String):
        """Callback to handle charging station status updates from monitor node."""
        try:
            data = json.loads(msg.data)
            self.charging_station_occupied = data.get('occupied', False)
            self.charging_queue = data.get('queue', [])
        except Exception as e:
            self.get_logger().error(f"[{self.robot_name}] Error parsing charging status: {e}")
    
    def other_pose_cb(self, msg: PoseStamped, robot_name: str):
        """Callback to track other robots' positions for collision avoidance."""
        self.other_poses[robot_name] = msg

    def spawn_cb(self, msg: SpawnedPackage):
        if not self.active_session:
            return
            
        with self._state_lock:
            # Don't bid if charging or waiting for charging
            if self.state != 'idle' or self.is_charging or self.waiting_for_charging:
                return # Busy doing something else
            if self.current_pose is None:
                return # Not initialized yet

            # Switch to bidding state IMMEDIATELY inside lock to avoid double-bidding
            self.state = 'bidding'
            
        self.publish_state()

        # Calculate cost
        rx = self.current_pose.pose.position.x
        rz = self.current_pose.pose.position.z
        tx = msg.shelf_position.x
        tz = msg.shelf_position.z
        
        dist_to_shelf = math.hypot(tx - rx, tz - rz)
        
        # Calculate cost with battery penalty
        base_cost = BID_DISTANCE_WEIGHT * dist_to_shelf
        battery_penalty = (1.0 - self.battery_level) * BATTERY_PENALTY_WEIGHT
        cost = base_cost + battery_penalty
        
        self.get_logger().info(f"[{self.robot_name}] Bid calculation: dist={dist_to_shelf:.2f}, battery={self.battery_level:.1%}, penalty={battery_penalty:.2f}")
        
        # Publish Bid
        bid = Bid()
        bid.robot_name = self.robot_name
        bid.task_id = msg.task_id
        bid.cost = float(cost)
        self.bid_pub.publish(bid)
        
        # Record my own bid
        if msg.task_id not in self.bids_received:
            self.bids_received[msg.task_id] = {}
        self.bids_received[msg.task_id][self.robot_name] = cost
        
        self.get_logger().info(f"[{self.robot_name}] Bid {cost:.2f} for {msg.task_id}")
        
        # Start auction wait in async loop
        asyncio.run_coroutine_threadsafe(self.evaluate_auction(msg), self.loop)

    async def evaluate_auction(self, package_msg: SpawnedPackage):
        # Wait for other bids
        await asyncio.sleep(BID_WAIT_TIME)
        
        # Check winner
        bids = self.bids_received.get(package_msg.task_id, {})
        if not bids:
            return # Should at least have my bid
            
        winner_name = min(bids, key=bids.get)
        
        if winner_name == self.robot_name:
            self.get_logger().info(f"[{self.robot_name}] WON auction for {package_msg.task_id}")
            await self.perform_delivery(package_msg)
        else:
            self.get_logger().info(f"[{self.robot_name}] LOST auction for {package_msg.task_id} to {winner_name}")
            with self._state_lock:
                 # Only reset to idle if we were bidding and lost, AND didn't start another task in the meantime (though unlikely with this logic)
                 if self.state == 'bidding':
                     self.state = 'idle'
            self.publish_state()

    # -----------------------
    # Action Server
    # -----------------------
    def goal_callback(self, goal_request):
        self.get_logger().info(f'[{self.robot_name}] Session request received')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info(f'[{self.robot_name}] Session cancel requested')
        return CancelResponse.ACCEPT

    def execute_session_callback(self, goal_handle):
        self.get_logger().info(f"[{self.robot_name}] Session STARTED")
        # Run async logic in the loop and wait for result
        future = asyncio.run_coroutine_threadsafe(self._async_execute_session(goal_handle), self.loop)
        return future.result()

    async def _async_execute_session(self, goal_handle):
        self.active_session = True
        self.expected_packages = goal_handle.request.expected_packages
        self.packages_delivered = 0
        
        feedback = ExecuteShift.Feedback()
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.active_session = False
                self.get_logger().info(f"[{self.robot_name}] Session CANCELED")
                return ExecuteShift.Result(success=True)
            
            feedback.delivered_packages = self.packages_delivered
            goal_handle.publish_feedback(feedback)
            
            await asyncio.sleep(1.0)
            
        goal_handle.succeed()
        return ExecuteShift.Result(success=True)

    # -----------------------
    # Task Execution
    # -----------------------
    async def perform_delivery(self, package: SpawnedPackage):
        with self._state_lock:
            self.state = 'retrieving'
        self.publish_state()
        
        # 1. Go to Shelf
        self.get_logger().info(f"[{self.robot_name}] Going to Shelf {package.shelf_id}...")
        success = await self.navigate_to(package.shelf_position)
        if not success:
            self.get_logger().error(f"[{self.robot_name}] Failed to reach shelf")
            with self._state_lock:
                self.state = 'idle'
            self.publish_state()
            return

        # 2. Pick up (simulate wait)
        await asyncio.sleep(1.0)
        # Switch to Delivering
        with self._state_lock:
            self.state = 'delivering'
        self.publish_state()
        
        # 3. Go to Package Destination
        delivery_pt = package.delivery_position
        self.get_logger().info(f"[{self.robot_name}] Going to Delivery at ({delivery_pt.x}, {delivery_pt.z})...")
        success = await self.navigate_to(delivery_pt)
        if not success:
            self.get_logger().error(f"[{self.robot_name}] Failed to reach delivery")
            with self._state_lock:
                self.state = 'idle'
            self.publish_state()
            return

        # 4. Drop off
        await asyncio.sleep(1.0)
        self.packages_delivered += 1
        self.get_logger().info(f"[{self.robot_name}] Delivered package {package.task_id} (battery: {self.battery_level:.1%})")
        # Switch to Returning (Hide package)
        with self._state_lock:
            self.state = 'returning'
        self.publish_state()
        
        # 5. Check if battery is low and needs charging
        if self.battery_level < LOW_BATTERY_THRESHOLD:
            self.get_logger().info(f"[{self.robot_name}] Battery low ({self.battery_level:.1%}), going to charge...")
            await self.go_to_charging_station()
        
        # Publish completion for metrics
        completion_msg = String()
        completion_msg.data = package.task_id
        self.task_completed_pub.publish(completion_msg)
        
        # 6. Return Home
        if self.home_pose:
            self.get_logger().info(f"[{self.robot_name}] Returning Home at ({self.home_pose.x}, {self.home_pose.z})...")
            await self.navigate_to(self.home_pose)
        
        with self._state_lock:
            self.state = 'idle'
        self.publish_state()

    async def navigate_to(self, target_point: Point):
        # Request path
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'map'
        pose_goal.pose.position = target_point
        pose_goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(pose_goal)
        
        # Wait for path
        self._path_waiter = self.loop.create_future()
        try:
            path = await asyncio.wait_for(self._path_waiter, timeout=5.0)
        except asyncio.TimeoutError:
            self.get_logger().error(f"[{self.robot_name}] Path planning timed out")
            return False
            
        # Follow path
        for waypoint in path.poses:
            if not await self.move_to_waypoint(waypoint):
                return False
        
        # Stop
        self.cmd_pub.publish(Twist())
        return True
    
    async def go_to_charging_station(self):
        """Navigate to charging station, wait in queue if occupied, then charge."""
        if self.charging_station_pos is None:
            self.get_logger().error(f"[{self.robot_name}] No charging station configured!")
            return
        
        with self._state_lock:
            self.state = 'charging'
            self.waiting_for_charging = True
        self.publish_state()
        
        # Request to enter charging queue
        request_msg = String()
        request_msg.data = json.dumps({'action': 'request', 'robot': self.robot_name})
        self.charging_request_pub.publish(request_msg)
        self.get_logger().info(f"[{self.robot_name}] Requested charging station...")
        
        # Wait until we are first in queue and station is not occupied
        while True:
            await asyncio.sleep(0.5)
            
            # Check if we are first in queue and station is free
            if len(self.charging_queue) > 0 and self.charging_queue[0] == self.robot_name:
                if not self.charging_station_occupied:
                    break
        
        self.waiting_for_charging = False
        self.is_charging = True
        
        # Notify that we are occupying the station
        occupy_msg = String()
        occupy_msg.data = json.dumps({'action': 'occupy', 'robot': self.robot_name})
        self.charging_request_pub.publish(occupy_msg)
        
        # Navigate to charging station
        self.get_logger().info(f"[{self.robot_name}] Navigating to charging station...")
        success = await self.navigate_to(self.charging_station_pos)
        if not success:
            self.get_logger().error(f"[{self.robot_name}] Failed to reach charging station")
            self.is_charging = False
            # Release the station
            release_msg = String()
            release_msg.data = json.dumps({'action': 'release', 'robot': self.robot_name})
            self.charging_request_pub.publish(release_msg)
            return
        
        # Charge for CHARGING_TIME seconds
        self.get_logger().info(f"[{self.robot_name}] Charging for {CHARGING_TIME}s...")
        await asyncio.sleep(CHARGING_TIME)
        
        # Battery fully charged
        self.battery_level = 1.0
        self.is_charging = False
        self.get_logger().info(f"[{self.robot_name}] Charging complete! Battery: {self.battery_level:.1%}")
        
        # Release the charging station
        release_msg = String()
        release_msg.data = json.dumps({'action': 'release', 'robot': self.robot_name})
        self.charging_request_pub.publish(release_msg)

    def get_robots_in_collision_range(self, radius: float = COLLISION_RADIUS) -> Dict[str, Tuple[float, float, float]]:
        """Get robots within collision range with their distance and position.
        
        Returns:
            Dict mapping robot_name -> (distance, other_x, other_z)
        """
        if self.current_pose is None:
            return {}
        
        rx = self.current_pose.pose.position.x
        rz = self.current_pose.pose.position.z
        nearby = {}
        
        for name, pose in self.other_poses.items():
            ox = pose.pose.position.x
            oz = pose.pose.position.z
            dist = math.hypot(ox - rx, oz - rz)
            if dist < radius:
                nearby[name] = (dist, ox, oz)
        
        return nearby
    
    def should_yield_to(self, other_name: str) -> bool:
        """Deterministic priority: lower robot name has priority (keeps moving)."""
        return self.robot_name > other_name
    
    def compute_avoidance_velocity(self, base_velocity: float, target_x: float, target_z: float, 
                                    nearby_robots: Dict[str, Tuple[float, float, float]]) -> Tuple[float, float, float]:
        """Compute adjusted velocity to avoid nearby robots while moving toward target.
        
        Returns:
            Tuple of (adjusted_velocity, adjusted_dx, adjusted_dz)
        """
        if self.current_pose is None:
            return base_velocity, target_x, target_z
        
        rx = self.current_pose.pose.position.x
        rz = self.current_pose.pose.position.z
        
        # Original direction to target
        dx = target_x - rx
        dz = target_z - rz
        dist_to_target = math.hypot(dx, dz)
        
        if dist_to_target < 0.01:
            return base_velocity, dx, dz
        
        # Normalize direction
        dx_norm = dx / dist_to_target
        dz_norm = dz / dist_to_target
        
        # Accumulate avoidance vectors
        avoid_x = 0.0
        avoid_z = 0.0
        
        for name, (dist, ox, oz) in nearby_robots.items():
            if dist < 0.1:
                dist = 0.1  # Avoid division by zero
            
            # Vector pointing away from other robot
            away_x = rx - ox
            away_z = rz - oz
            away_len = math.hypot(away_x, away_z)
            if away_len > 0.01:
                # Weight by inverse distance (closer = stronger avoidance)
                weight = (COLLISION_SLOW_RADIUS - dist) / COLLISION_SLOW_RADIUS
                weight = max(0, min(1, weight))
                avoid_x += (away_x / away_len) * weight
                avoid_z += (away_z / away_len) * weight
        
        # Blend original direction with avoidance
        blend_factor = 0.4  # How much to blend avoidance
        final_dx = dx_norm + avoid_x * blend_factor
        final_dz = dz_norm + avoid_z * blend_factor
        
        # Re-normalize
        final_len = math.hypot(final_dx, final_dz)
        if final_len > 0.01:
            final_dx /= final_len
            final_dz /= final_len
        
        # Reduce velocity based on closest robot
        if nearby_robots:
            min_dist = min(d[0] for d in nearby_robots.values())
            # Linear velocity reduction: full speed at SLOW_RADIUS, MIN_VELOCITY at COLLISION_RADIUS
            speed_factor = (min_dist - COLLISION_RADIUS) / (COLLISION_SLOW_RADIUS - COLLISION_RADIUS)
            speed_factor = max(0.0, min(1.0, speed_factor))
            adjusted_velocity = MIN_VELOCITY + (base_velocity - MIN_VELOCITY) * speed_factor
        else:
            adjusted_velocity = base_velocity
        
        return adjusted_velocity, final_dx * dist_to_target, final_dz * dist_to_target

    async def move_to_waypoint(self, waypoint, tol=0.2):
        target = waypoint.pose.position
        
        # Control loop frequency
        dt = 0.05
        base_velocity = 2.0
        
        # Reset yield tracking
        self.yield_start_time = None
        
        while True:
            if self.current_pose is None:
                await asyncio.sleep(0.1)
                continue
            
            rx = self.current_pose.pose.position.x
            rz = self.current_pose.pose.position.z
            
            dx = target.x - rx
            dz = target.z - rz
            dist = math.hypot(dx, dz)

            if dist < tol:
                self.yield_start_time = None
                return True
            
            # Get robots in collision range
            nearby_collision = self.get_robots_in_collision_range(COLLISION_RADIUS)
            nearby_slow = self.get_robots_in_collision_range(COLLISION_SLOW_RADIUS)
            
            # Check if we need to yield to any robot in hard collision range
            should_stop = False
            if nearby_collision:
                # Only yield to robots with higher priority (lower name)
                higher_priority_nearby = [n for n in nearby_collision if self.should_yield_to(n)]
                
                if higher_priority_nearby:
                    # We should yield - but check for deadlock timeout
                    if self.yield_start_time is None:
                        self.yield_start_time = time.time()
                    
                    elapsed_yield = time.time() - self.yield_start_time
                    
                    if elapsed_yield > YIELD_TIMEOUT:
                        # Deadlock recovery: random backoff then try micro-movement
                        self.get_logger().warn(f'[{self.robot_name}] Yield timeout! Attempting deadlock recovery...')
                        self.cmd_pub.publish(Twist())  # Stop
                        await asyncio.sleep(random.uniform(0.1, RANDOM_BACKOFF_MAX))
                        self.yield_start_time = time.time()  # Reset timer
                        # Don't stop - try to inch forward slowly
                        should_stop = False
                    else:
                        should_stop = True
            else:
                # No collision - reset yield timer
                self.yield_start_time = None
            
            if should_stop:
                # Full stop - yield to higher priority robot
                self.cmd_pub.publish(Twist())
                await asyncio.sleep(COLLISION_WAIT_TIME)
                continue
            
            # Compute velocity with avoidance for robots in slow range
            if nearby_slow:
                linear_v, adj_dx, adj_dz = self.compute_avoidance_velocity(
                    base_velocity, target.x, target.z, nearby_slow
                )
                # Recalculate direction from adjusted values
                adj_dist = math.hypot(adj_dx, adj_dz)
                if adj_dist > 0.01:
                    target_yaw = math.atan2(adj_dz, adj_dx)
                else:
                    target_yaw = math.atan2(dz, dx)
            else:
                linear_v = base_velocity
                target_yaw = math.atan2(dz, dx)
            
            # Update internal state (Dead Reckoning)
            # We move towards the target
            move_dist = linear_v * dt
            if move_dist > dist:
                move_dist = dist
            
            # Drain battery based on distance moved
            self.battery_level -= move_dist * BATTERY_DRAIN_PER_METER
            if self.battery_level < 0:
                self.battery_level = 0
                
            self.current_yaw = target_yaw
            
            rx += move_dist * math.cos(self.current_yaw)
            rz += move_dist * math.sin(self.current_yaw)
            
            # Update pose
            self.current_pose.pose.position.x = rx
            self.current_pose.pose.position.z = rz
            
            # Quaternion from yaw
            cy = math.cos(self.current_yaw * 0.5)
            sy = math.sin(self.current_yaw * 0.5)
            self.current_pose.pose.orientation.w = cy
            self.current_pose.pose.orientation.z = sy
            self.current_pose.pose.orientation.x = 0.0
            self.current_pose.pose.orientation.y = 0.0
            
            self.current_pose.header.stamp = self.get_clock().now().to_msg()
            self.pose_pub.publish(self.current_pose)

            # Publish cmd_vel (for visualization or other nodes if needed)
            twist = Twist()
            twist.linear.x = linear_v
            # twist.angular.z = ... 
            self.cmd_pub.publish(twist)
            
            await asyncio.sleep(dt)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
