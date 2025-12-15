#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, Bool
import math, heapq, time
from typing import List, Tuple, Optional

class PathPlannerServer(Node):
    def __init__(self):
        super().__init__('path_planner_server')

        # parameter: single robot name
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # internal map
        self.map_msg: Optional[OccupancyGrid] = None
        self.map_received = False

        # per-robot publishers/subscribers and last known pose
        self.robot_pose: Optional[PoseStamped] = None

        # QoS: accept transient_local for map if available
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        map_qos.durability = QoSDurabilityPolicy.VOLATILE

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile=map_qos)
        self.get_logger().info('Subscribed to /map (waiting for map messages)...')

        # Create per-robot topics
        goal_topic = f'/{self.robot_name}/goal'
        pose_topic = f'/{self.robot_name}/pose'    # optional: robot publishes its current pose here
        path_topic = f'/{self.robot_name}/path'
        ready_topic = f'/{self.robot_name}/plan_ready'

        # publisher for path
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        
        # publisher for readiness (latched)
        qos_latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, ready_topic, qos_profile=qos_latched)

        # subscriber for goal: when robot asks for path to a goal
        self.create_subscription(PoseStamped, goal_topic, self.goal_cb, 10)
        # optional: subscribe to robot pose to have start (if robots publish)
        self.create_subscription(PoseStamped, pose_topic, self.pose_cb, 10)

        self.get_logger().info(f'PathPlannerServer initialized for {self.robot_name}.')
        self.get_logger().info(f'Topics: goal={goal_topic}, pose={pose_topic}, path={path_topic}, ready={ready_topic}')

    # --- map handling ---
    def map_callback(self, msg: OccupancyGrid):
        # Always store the latest map
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        origin = msg.info.origin.position
        occ = sum(1 for v in msg.data if v > 50) if msg.data else 0
        total = w * h if w and h else 0
        occ_pct = (occ / total * 100.0) if total else 0.0

        # If map empty → ignore
        if occ == 0:
            self.get_logger().warn("Received EMPTY map, ignoring.")
            return

        # Store valid (non-empty) map
        self.map_msg = msg
        self.map_received = True

        self.get_logger().info(
            f"Stored valid map: {w}x{h} res={res:.3f} occupied={occ}/{total} ({occ_pct:.1f}%)"
        )
        
        # Signal readiness
        ready_msg = Bool()
        ready_msg.data = True
        self.ready_pub.publish(ready_msg)


    # --- per-robot pose update ---
    def pose_cb(self, msg: PoseStamped):
        self.robot_pose = msg
        # debug-level log (frequent)
        self.get_logger().debug(f'[{self.robot_name}] pose update: ({msg.pose.position.x:.2f},{msg.pose.position.z:.2f})')

    # --- per-robot goal callback ---
    def goal_cb(self, msg: PoseStamped):
        self.get_logger().info(f'[{self.robot_name}] Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.z:.2f})')
        if not self.map_received:
            self.get_logger().warning(f'[{self.robot_name}] No map available - cannot plan')
            return
        # determine start: prefer robot_pose if present, else use msg.header (or fail)
        if self.robot_pose:
            start_pose = self.robot_pose.pose
        else:
            # fallback: use goal header frame as start? better to warn and fail
            self.get_logger().warning(f'[{self.robot_name}] No start pose available (no /{self.robot_name}/pose). Cannot plan.')
            return

        t0 = time.time()
        cells = self.plan_a_star(start_pose, msg.pose)
        dt = time.time() - t0

        if cells is None:
            self.get_logger().warning(f'[{self.robot_name}] A* failed to find a path (t={dt:.3f}s)')
            return

        path_msg = self.cells_to_path(cells)
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'[{self.robot_name}] Path published: {len(cells)} waypoints (computed in {dt:.3f}s)')

    # --- grid helpers ---
    def world_to_grid(self, x: float, z: float) -> Tuple[int,int]:
        origin = self.map_msg.info.origin.position
        res = self.map_msg.info.resolution
        gx = int(math.floor((x - origin.x) / res))
        gz = int(math.floor((z - origin.z) / res))
        gx = max(0, min(gx, self.map_msg.info.width - 1))
        gz = max(0, min(gz, self.map_msg.info.height - 1))
        return gx, gz

    def grid_to_world(self, gx:int, gz:int) -> Tuple[float,float]:
        origin = self.map_msg.info.origin.position
        res = self.map_msg.info.resolution
        x = origin.x + (gx + 0.5) * res
        z = origin.z + (gz + 0.5) * res
        return x, z

    def is_occupied(self, gx:int, gz:int) -> bool:
        idx = gz * self.map_msg.info.width + gx
        val = self.map_msg.data[idx]
        return val > 50

    def neighbors(self, gx:int, gz:int):
        for dx, dz in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nx, nz = gx+dx, gz+dz
            if 0 <= nx < self.map_msg.info.width and 0 <= nz < self.map_msg.info.height:
                if not self.is_occupied(nx, nz):
                    yield nx, nz

    def heuristic(self, a:Tuple[int,int], b:Tuple[int,int]) -> float:
        return math.hypot(b[0]-a[0], b[1]-a[1])

    # --- A* implementation ---
    def plan_a_star(self, start_pose: Pose, goal_pose: Pose) -> Optional[List[Tuple[int,int]]]:
        sx, sz = start_pose.position.x, start_pose.position.z
        gx, gz = goal_pose.position.x, goal_pose.position.z
        start = self.world_to_grid(sx, sz)
        goal = self.world_to_grid(gx, gz)
        self.get_logger().debug(f'A*: start_cell={start} goal_cell={goal}')

        # quick check occupancy
        if self.is_occupied(goal[0], goal[1]):
            self.get_logger().warning('A* goal cell occupied -> abort')
            return None
        if self.is_occupied(start[0], start[1]):
            self.get_logger().warning('A* start cell occupied -> attempting plan but start is occupied')

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))
        came_from = {}
        gscore = {start: 0.0}
        fscore = {start: self.heuristic(start, goal)}
        closed = set()
        max_iters = self.map_msg.info.width * self.map_msg.info.height * 2
        iters = 0

        while open_heap:
            iters += 1
            if iters > max_iters:
                self.get_logger().warning('A* exceeded max iterations')
                break
            _, current = heapq.heappop(open_heap)
            if current == goal:
                # reconstruct
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            closed.add(current)
            for nb in self.neighbors(*current):
                if nb in closed:
                    continue
                tentative_g = gscore[current] + self.heuristic(current, nb)
                if tentative_g < gscore.get(nb, float('inf')):
                    came_from[nb] = current
                    gscore[nb] = tentative_g
                    f = tentative_g + self.heuristic(nb, goal)
                    fscore[nb] = f
                    heapq.heappush(open_heap, (f, nb))
        return None

    # --- convert cells to nav_msgs/Path ---
    def cells_to_path(self, cells: List[Tuple[int,int]]) -> Path:
        path = Path()
        poses = []
        for gx, gz in cells:
            x,z = self.grid_to_world(gx, gz)
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'map'
            ps.pose = Pose()
            ps.pose.position = Point(x=x, y=0.0, z=z)
            ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            poses.append(ps)
        path.poses = poses
        return path

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
