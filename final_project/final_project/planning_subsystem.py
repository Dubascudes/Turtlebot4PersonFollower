from tf2_ros import Buffer, TransformListener, TransformException
import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import Bool, UInt8, String, Int16, Float32
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, TwistStamped
import math
from rclpy.time import Time
from tf_transformations import quaternion_from_euler
from irobot_create_msgs.msg import DockStatus
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance, RotateAngle, Dock, Undock, NavigateToPosition

from enum import Enum
class State(Enum):
    IDLE      = 0
    FOLLOWING = 1

class PlannerNode(Node):
    """High-level finite-state planner.
    Consumes perception topics and emits goal/command topics for the controller.
    """
    def __init__(self):
        super().__init__('planner_subsystem')
        # # ───── parameters ────────────────────────────────────────────
        self.declare_parameter('home_yaw_offset', math.pi)
        self.home_offset   = self.get_parameter('home_yaw_offset').value

        # Internal state
        self.state = State.IDLE
        self.target_detected   = False
        self.last_target_seen  = self.get_clock().now().seconds_nanoseconds()[0]
        self.last_nav_time     = 0.0
        self.last_target_pose  = None  # (range, bearing)
        
        # Command tracking
        self.dock_command_sent = False
        self.return_command_sent = False
        self.undock_command_sent = False
        self.is_docked = False


        self.nav_client    = ActionClient(self, NavigateToPosition, '/navigate_to_position')
        self.dock_client   = ActionClient(self, Dock, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.rotate_client = ActionClient(self, RotateAngle,   '/rotate_angle')
        self.drive_client  = ActionClient(self, DriveDistance, '/drive_distance')  # ⬅ new
        # Perception inputs
        self.create_subscription(Bool, '/see/target_in_frame', self._target_flag_cb, 10)
        self.create_subscription(PoseStamped, '/see/person_location', self._target_pose_cb, 10)
        self.create_subscription(Float32, '/see/target_distance', self._target_distance_cb, 10)
        self.create_subscription(Float32, '/see/target_heading', self._target_heading_cb, 10)

        # CLI inputs
        self.create_subscription(String, '/think/ctrl_cmd', self._cli_cb, 10)

        # Dock status
        self.create_subscription(DockStatus, '/dock_status', self._dock_status_cb, 10)

        # TF helper
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.dock_pose_stamped = None
        self.create_timer(1.0, self._capture_home_pose)

        # Action client for direct rotation control
        self.rotate_client = ActionClient(self, RotateAngle, '/rotate_angle')
        
        # Planner outputs consumed by CLI
        self.state_pub     = self.create_publisher(UInt8,       '/think/planner_state', 10)


        self.cmd_vel_pub = self.create_publisher(
                TwistStamped,                                
                '/cmd_vel',                           
                10)

        self.create_timer(0.1, self._fsm_step)
        
        self.target_pose = None
        self.target_distance = 0.0
        self.target_heading = 0.0


        rclpy.get_global_executor().create_task(self._connect_undock_server())
        rclpy.get_global_executor().create_task(self._connect_dock_server())
        # ─── in __init__ ───────────────────────────────────────────────────
        self.search_rot_speed = 1.9         # [rad s-1]  spin while searching
        self._last_seen_bearing = 0.0       # updated every frame the target is visible

        self.dead_ang   = math.radians(3)   # ignore ±3°
        self.smooth_fac = 0.2               # 0–1  (low-pass on ω)
        self._prev_w    = 0.0               # remember last command
        self.follow_thresh   = 1.0    # m – start walking toward person
        self.kp_lin      = 0.5      # [-]  proportional gain for range      (tuned)
        self.kp_ang      = 0.7     # [-]  proportional gain for bearing    (tuned)
        self.max_lin_vel = 0.55     # [m/s] absolute speed limits
        self.max_ang_vel = 1.90     # [rad/s]

        self.get_logger().info('Planner subsystem started')

        self._last_cmd = None


    def _publish_cmd_vel(self, v: float, w: float):
        msg                  = TwistStamped()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'base_link'          # any fixed frame is fine
        msg.twist.linear.x   = max(-self.max_lin_vel,  min(self.max_lin_vel,  v))
        msg.twist.angular.z  = max(-self.max_ang_vel, min(self.max_ang_vel, w))
        self.cmd_vel_pub.publish(msg)
        self._last_cmd = (msg.twist.linear.x, msg.twist.angular.z)

    # --------------------------------------------------------------------- CLI
    def _cli_cb(self, msg: String):
        if msg.data == 'stop':
            self.get_logger().info('Stop command received           State = Idle')
            self.state = State.IDLE

        elif msg.data == 'follow':
            self.get_logger().info('Follow command received         State = Following')
            self.state = State.FOLLOWING

        elif msg.data == 'dock':
            self.get_logger().info('Dock command received           State = Idle            Sending Dock Goal')
            self.state = State.IDLE
            self._send_dock()

        elif msg.data == 'undock':
            self.get_logger().info('Unock command received           State = Idle            Sending Undock Goal')
            self.state = State.IDLE
            self._send_undock()

    # --------------------------------------------------------------------- FSM
    def _fsm_step(self):
        # # ---------------------------------------------------- P-Controller for Following
        if self.state == State.FOLLOWING:
            # ---------------------------------- if the target is *not* in view
            if not self.target_detected:
                # choose spin direction from the last bearing sign (+left, –right)
                spin_dir =  1.0 if self._last_seen_bearing >= 0 else -1.0
                self._publish_cmd_vel(0.0,              # no forward motion
                                    spin_dir * self.search_rot_speed)
                return                                   # skip normal P-control

            # ---------------------------------- normal P control (target visible)
            # remember where we see them *now* for future spins
            self._last_seen_bearing = self.target_heading

            range_error = self.target_distance - self.follow_thresh
            ang_error   = self.target_heading

            v_raw = self.kp_lin * range_error
            w_raw = self.kp_ang * ang_error

            if abs(ang_error) < self.dead_ang:
                w_raw = 0.0

            v_raw *= max(0.0, math.cos(ang_error))

            w = self.smooth_fac * w_raw + (1 - self.smooth_fac) * self._prev_w
            self._prev_w = w

            v = max(0.0, v_raw)
            self._publish_cmd_vel(v, w)

        self.state_pub.publish(UInt8(data=self.state.value))

    async def _connect_undock_server(self):
        """Wait for the RotateAngle action server to become available"""
        while not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Undock action server...')
        self.undock_available = True
        self.get_logger().info('Connected to Undock action server')

    async def _connect_dock_server(self):
        """Wait for the RotateAngle action server to become available"""
        while not self.dock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Dock action server...')
        self.dock_available = True
        self.get_logger().info('Connected to Dock action server')

    # May be used to navigate near the dock home; currently unused
    # ───────────── TF home pose ─────────────
    def _capture_home_pose(self):
        if self.dock_pose_stamped is not None:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            yaw = math.atan2(2*(tf.transform.rotation.w*tf.transform.rotation.z +
                                tf.transform.rotation.x*tf.transform.rotation.y),
                              1 - 2*(tf.transform.rotation.y**2 + tf.transform.rotation.z**2))
            opp = (yaw + self.home_offset) % (2*math.pi) - math.pi
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            q = quaternion_from_euler(0, 0, opp)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            self.dock_pose_stamped = pose
            self.get_logger().info('Captured dock pose')
        except TransformException:
            pass

    def _publish_home_pose(self):
        if self.dock_pose_stamped is not None:
            self.dock_pub.publish(self.dock_pose_stamped)
            
    # --------------------------------------------------------------------- dock status callback
    def _dock_status_cb(self, msg: DockStatus):
        self.is_docked = msg.is_docked
            
    # --------------------------------------------------------------------- perception callbacks
    def _target_flag_cb(self, msg: Bool):
        self.target_detected = msg.data
        if msg.data:
            self.last_target_seen = self.get_clock().now().seconds_nanoseconds()[0]

    def _target_pose_cb(self, msg: PoseStamped):
        self.target_pose = msg

    def _target_distance_cb(self, msg: Float32):
        self.target_distance = msg.data


    def _target_heading_cb(self, msg: Float32):
        self.target_heading = msg.data


    def _send_undock(self):
        if not self.is_docked:
            self.get_logger().warn('Undock requested but robot is not docked')
            return
        self.get_logger().info('Undock request sent')

        self.undock_client.send_goal_async(Undock.Goal())

    def _send_dock(self):
        if self.is_docked:
            self.get_logger().warn('Dock requested but robot is already docked')
            return
        self.get_logger().info('Dock request sent')
        self.dock_client.send_goal_async(Dock.Goal())

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()