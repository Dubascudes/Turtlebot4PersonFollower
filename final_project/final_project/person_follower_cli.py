#!/usr/bin/env python3
"""
Terminal dashboard / teleop for the person‑follower stack
· Shows current state + target polar estimate
· Sends simple string commands on '/person_follower/cmd'
"""
from tf2_ros import Buffer, TransformListener, TransformException
import curses, math, rclpy, threading, time
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import UInt8, String, Float32
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

_STATES = {0: "IDLE", 1: "FOLLOWING"}

class Dashboard(Node):
    def __init__(self, stdscr):
        super().__init__("person_follower_cli")
        self.stdscr = stdscr
        self.state = 0
        self.target_x = 0.0 #float("nan")
        self.target_y = 0.0 #float("nan")
        self.map_x = 0.0    # Position in map frame
        self.map_y = 0.0    # Position in map frame
        self.last_command = 0
        qos = QoSProfile(   
            depth=  10,
            reliability= ReliabilityPolicy.RELIABLE,
            durability=   DurabilityPolicy.VOLATILE
        )
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(UInt8, '/think/planner_state', self._state_cb, 10)

        # subscribe to both base_link and map frame positions
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseStamped, '/see/person_location', self._target_cb, qos)
        self.create_subscription(PoseStamped, '/see/person_location_map', self._map_target_cb, qos)
        self.create_subscription(Float32, '/see/target_distance', self._target_distance_cb, 10)
        self.create_subscription(Float32, '/see/target_heading', self._target_heading_cb, 10)

        self.cmd_pub    = self.create_publisher(String, '/think/ctrl_cmd', 10)
        self.marker_pub = self.create_publisher(Marker, '/person_marker', 10)

        # 1) spin ROS in background so callbacks run
        threading.Thread(target=lambda: rclpy.spin(self), daemon=True).start()

        # 2) spawn your curses input loop
        threading.Thread(target=self._key_loop, daemon=True).start()

        self.target_distance = 0.0
        self.target_heading = 0.0
    def _state_cb(self, msg):   self.state = msg.data

    def _target_distance_cb(self, msg: Float32):
        self.target_distance = msg.data


    def _target_heading_cb(self, msg: Float32):
        self.target_heading = msg.data

    def _target_cb(self, msg):
        # Store base_link frame position
        self.target_x = msg.pose.position.x 
        self.target_y = msg.pose.position.y
        
    def _map_target_cb(self, msg):
        # Store map frame position
        self.map_x = msg.pose.position.x
        self.map_y = msg.pose.position.y
        
    # ------------------ Keyboard loop ------------------
    def _key_loop(self):
        # curses in raw mode: non‑blocking getch()
        self.stdscr.nodelay(True)
        while rclpy.ok():
            key = self.stdscr.getch()
            if key == -1:
                time.sleep(0.05)
                continue

            if key in (ord('q'), 27):         # q or Esc quits
                rclpy.shutdown();  break
            elif key == ord('s'):             # stop immediately
                self.cmd_pub.publish(String(data="stop"))
            elif key == ord('f'):             # resume following
                self.cmd_pub.publish(String(data="follow"))


            elif key == ord('d'):             # dock without navigation
                self.cmd_pub.publish(String(data="dock"))
            # elif key == ord('g'):             # return to dock (navigate to dock pose)
            #     self.cmd_pub.publish(String(data="return_to_dock"))
            elif key == ord('u'):             # restart search
                # 1) check that we have real numbers
                self.cmd_pub.publish(String(data="undock"))
    # ------------------ Screen refresh -----------------
    def draw(self):
        h, w = self.stdscr.getmaxyx()        # current window size
        self.stdscr.erase()

        def put(y, x, text):
            """Write text if it fits on the screen."""
            if 0 <= y < h and x < w:
                self.stdscr.addnstr(y, x, text, w - x - 1)   # clip at rhs

        put(0, 0, "Person-Follower CLI  –  press [q] to quit")
        put(2, 0, f"STATE   : {_STATES.get(self.state,'?')}")
        put(3, 0, f"BASE_LINK X: {self.target_x:5.1f}")
        put(4, 0, f"BASE_LINK Y: {self.target_y:5.1f}")
        put(6, 0,
            f"Distance: {self.target_distance:5.1f}  "
            f"Bearing: {math.degrees(self.target_heading):5.1f}°")
        put(7, 0, f"Last Command: {_STATES.get(self.last_command,'-')}")
        put(8, 0,
            "[s] Idle   [f] Follow   [d] Dock   [u] Undock")

        self.stdscr.refresh()

def  _curses_main(stdscr):
    rclpy.init()
    dash = Dashboard(stdscr)
    try:
        while rclpy.ok():
            dash.draw()
            time.sleep(1.0)
    finally:
        dash.destroy_node()
        rclpy.shutdown()

def main():
    curses.wrapper(_curses_main)

if __name__ == "__main__":
    main()
