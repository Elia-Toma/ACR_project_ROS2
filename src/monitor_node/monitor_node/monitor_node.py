#!/usr/bin/env python3
import os
import time
import datetime
import random
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String
from robot_interfaces.msg import SpawnedPackage, Bid
from robot_interfaces.action import ExecuteShift

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        # Parameters
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        self.declare_parameter('spawn_interval', 5.0)
        self.declare_parameter('min_packages_per_shelf', 2)
        self.declare_parameter('max_packages_per_shelf', 5)
        
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        self.spawn_interval = self.get_parameter('spawn_interval').get_parameter_value().double_value
        self.min_pkgs = self.get_parameter('min_packages_per_shelf').get_parameter_value().integer_value
        self.max_pkgs = self.get_parameter('max_packages_per_shelf').get_parameter_value().integer_value

        # Publishers
        self.spawn_pub = self.create_publisher(SpawnedPackage, '/spawned_package', 10)

        # Action Clients
        self.action_clients = {}
        for name in self.robot_names:
            self.action_clients[name] = ActionClient(self, ExecuteShift, f'/{name}/execute_shift')

        # State
        self.packages_spawned = 0
        self.total_packages = 0
        self.total_delivered = 0
        self.robot_delivered = {name: 0 for name in self.robot_names}
        
        # Charging station state
        self.charging_queue = []  # Queue of robot names waiting to charge
        self.station_occupied_by = None  # Robot currently charging
        
        # Synchronization State
        self.robot_ready = {name: False for name in self.robot_names}
        self.config_received = False
        self.simulation_started = False

        # Subscribe to plan_ready for each robot
        for name in self.robot_names:
            topic = f'/{name}/plan_ready'
            self.create_subscription(Bool, topic, self._make_ready_cb(name), 10)
            self.get_logger().info(f"Waiting for {topic}...")

        # Subscribe to configuration
        self.create_subscription(String, '/simulation/config', self.config_callback, 10)
        self.get_logger().info("Waiting for /simulation/config...")
        
        # Charging station status publisher
        self.charging_status_pub = self.create_publisher(String, '/charging_station/status', 10)
        
        # Subscribe to charging requests from each robot
        for name in self.robot_names:
            topic = f'/{name}/charging_request'
            self.create_subscription(String, topic, self._make_charging_request_cb(name), 10)
            self.get_logger().info(f"Listening for charging requests on {topic}")

        # Shelves and delivery points will be loaded from config
        self.shelves = []
        self.delivery_points = []  # List of Point objects
        # Package tracking for reliability
        self.pending_tasks = {} # task_id -> {'msg': msg, 'ts': time.time()}
        self.create_subscription(Bid, '/robot_bids', self.bid_cb, 10)
        self.create_subscription(String, '/task_completed', self.task_completed_cb, 10)
        self.create_timer(2.0, self.check_pending_tasks)

        # Metrics
        self.spawn_times = {} # task_id -> timestamp
        self.delivery_durations = []
        self.simulation_start_time = None
        self.sim_logs = []

    def log_event(self, message):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.sim_logs.append(log_entry)
        self.get_logger().info(message)

    def config_callback(self, msg: String):
        if self.config_received:
            return
        
        try:
            data = json.loads(msg.data)
            # Expected format: {"shelves": [{"id": 1, "x": 2.0, "z": 2.0, "count": 5}, ...], ...}
            
            if 'shelves' not in data:
                self.get_logger().error("Config missing 'shelves' key")
                return

            self.shelves = []
            self.total_packages = 0
            
            for s in data['shelves']:
                shelf = {
                    'id': s['id'],
                    'pos': Point(x=float(s['x']), y=0.0, z=float(s['z'])),
                    'count': s['count']
                }
                self.shelves.append(shelf)
                self.total_packages += shelf['count']
            
            self.config_received = True
            
            dp_count = 0
            if 'delivery_points' in data:
                self.delivery_points = []
                for dp in data['delivery_points']:
                    self.delivery_points.append(Point(x=float(dp['x']), y=0.0, z=float(dp['z'])))
                dp_count = len(self.delivery_points)
            elif 'delivery' in data:
                d = data['delivery']
                self.delivery_points = [Point(x=float(d['x']), y=0.0, z=float(d['z']))]
                dp_count = 1
                
            self.log_event(f"Config received. Loaded {len(self.shelves)} shelves and {dp_count} delivery points. Total packages: {self.total_packages}")
            self.check_start_simulation()
            
        except json.JSONDecodeError:
            self.get_logger().error("Failed to decode JSON config")
        except Exception as e:
            self.get_logger().error(f"Error processing config: {e}")

    def _make_ready_cb(self, robot_name):
        def cb(msg: Bool):
            if msg.data:
                if not self.robot_ready[robot_name]:
                    self.robot_ready[robot_name] = True
                    self.log_event(f"Robot {robot_name} is READY (map received).")
                    self.check_start_simulation()
        return cb

    def check_start_simulation(self):
        if self.simulation_started:
            return
        
        # Condition: All robots ready (map) AND Config received
        if all(self.robot_ready.values()) and self.config_received:
            self.log_event("All robots ready and config received. Starting simulation...")
            self.start_simulation()

    def start_simulation(self):
        self.simulation_started = True
        self.log_event("Starting simulation logic...")
        
        # Send Goal to all robots
        for name, client in self.action_clients.items():
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"Action server for {name} not available!")
                continue
            
            goal = ExecuteShift.Goal()
            goal.shift_id = 1
            goal.expected_packages = self.total_packages
            goal.battery_initial = 1.0
            
            self.log_event(f"Sending ExecuteShift goal to {name}...")
            
            future = client.send_goal_async(goal, feedback_callback=self._make_feedback_cb(name))
            future.add_done_callback(lambda f, n=name: self.goal_response_callback(f, n))

        # Start Spawning Timer
        self.create_timer(self.spawn_interval, self.spawn_package)
        self.simulation_start_time = time.time()

    def _make_feedback_cb(self, robot_name):
        def cb(feedback_msg):
            # feedback_msg.feedback contains the feedback fields
            delivered = feedback_msg.feedback.delivered_packages
            self.robot_delivered[robot_name] = delivered
            self.update_total_progress()
        return cb

    def update_total_progress(self):
        current_total = sum(self.robot_delivered.values())
        if current_total != self.total_delivered:
            self.total_delivered = current_total
            self.get_logger().info(f"Progress Update: {self.total_delivered}/{self.total_packages} packages delivered.")
            if self.total_delivered >= self.total_packages:
                self.generate_report()

    def goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal rejected by {robot_name}")
            return
        self.log_event(f"Goal accepted by {robot_name}")

    def spawn_package(self):
        # Find shelves with remaining packages
        available_shelves = [s for s in self.shelves if s['count'] > 0]
        
        if not available_shelves:
            if self.packages_spawned < self.total_packages:
                # Should not happen if logic is correct
                pass
            else:
                # All packages spawned
                pass
            return

        shelf = random.choice(available_shelves)
        shelf['count'] -= 1
        
        # Choose random delivery point
        delivery_point = random.choice(self.delivery_points) if self.delivery_points else Point(x=0.0, y=0.0, z=0.0)
        
        msg = SpawnedPackage()
        msg.task_id = f"pkg_{self.packages_spawned + 1}"
        msg.shelf_id = shelf['id']
        msg.shelf_position = shelf['pos']
        msg.delivery_position = delivery_point
        
        self.spawn_pub.publish(msg)
        
        # Track pending task
        self.pending_tasks[msg.task_id] = {'msg': msg, 'ts': time.time()}
        self.spawn_times[msg.task_id] = time.time()
        
        self.packages_spawned += 1
        self.log_event(f"Spawned package {msg.task_id} at Shelf {msg.shelf_id} ({shelf['count']} left there)")
    
    def bid_cb(self, msg: Bid):
        if msg.task_id in self.pending_tasks:
            del self.pending_tasks[msg.task_id]
            # self.get_logger().info(f"Task {msg.task_id} accepted, removed from pending.")
    def check_pending_tasks(self):
        now = time.time()
        timeout = 30.0 # seconds (User requested)
        
        for task_id, data in list(self.pending_tasks.items()):
            if now - data['ts'] > timeout:
                self.get_logger().warn(f"Task {task_id} timed out (no bids). Republishing...")
                self.spawn_pub.publish(data['msg'])
                # Reset timestamp to avoid spamming every 2s, wait another 15s
                data['ts'] = now

    def task_completed_cb(self, msg: String):
        task_id = msg.data
        if task_id in self.spawn_times:
            duration = time.time() - self.spawn_times[task_id]
            self.delivery_durations.append(duration)
            self.log_event(f"Task {task_id} completed in {duration:.2f}s")
    
    def generate_report(self):
        self.get_logger().info("Simulation Finished! Generating report...")
        
        total_time = time.time() - self.simulation_start_time
        avg_delivery = sum(self.delivery_durations) / len(self.delivery_durations) if self.delivery_durations else 0.0
        
        timestamp_str = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
        filename = f"run_{timestamp_str}.txt"
        
        # Ensure output directory exists
        # Assuming we are running from src/monitor_node typically, or we can just use relative path 'output'
        # which will be in the cwd (likely ~/.ros or the launch dir). 
        # User requested "output" folder.
        output_dir = "output"
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        filepath = os.path.join(output_dir, filename)
        
        with open(filepath, "w") as f:
            f.write("Simulation Report\n")
            f.write("=================\n\n")
            f.write(f"Total Simulation Time: {total_time:.2f} s\n")
            f.write(f"Total Packages Delivered: {self.total_delivered}\n")
            f.write(f"Average Package Delivery Time: {avg_delivery:.2f} s\n\n")
            f.write("Robot Performance:\n")
            for name, count in self.robot_delivered.items():
                f.write(f"  {name}: {count} packages\n")

            f.write("\nEvent Log:\n")
            f.write("==========\n")
            for log in self.sim_logs:
                f.write(f"{log}\n")
        
        self.get_logger().info(f"Report saved to {filepath}")
    
    def _make_charging_request_cb(self, robot_name):
        """Create a callback for handling charging requests from a specific robot."""
        def cb(msg: String):
            try:
                data = json.loads(msg.data)
                action = data.get('action')
                robot = data.get('robot', robot_name)
                
                if action == 'request':
                    # Robot wants to join the charging queue
                    if robot not in self.charging_queue:
                        self.charging_queue.append(robot)
                        self.log_event(f"[Charging] {robot} joined queue. Queue: {self.charging_queue}")
                        self.publish_charging_status()
                
                elif action == 'occupy':
                    # Robot is now using the station
                    if robot in self.charging_queue:
                        self.station_occupied_by = robot
                        self.log_event(f"[Charging] {robot} is now charging")
                        self.publish_charging_status()
                
                elif action == 'release':
                    # Robot finished charging
                    if robot in self.charging_queue:
                        self.charging_queue.remove(robot)
                    if self.station_occupied_by == robot:
                        self.station_occupied_by = None
                    self.log_event(f"[Charging] {robot} released station. Queue: {self.charging_queue}")
                    self.publish_charging_status()
                    
            except Exception as e:
                self.get_logger().error(f"Error processing charging request: {e}")
        return cb
    
    def publish_charging_status(self):
        """Publish the current charging station status."""
        status = {
            'occupied': self.station_occupied_by is not None,
            'occupied_by': self.station_occupied_by,
            'queue': self.charging_queue
        }
        msg = String()
        msg.data = json.dumps(status)
        self.charging_status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
