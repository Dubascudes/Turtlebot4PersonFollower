import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan, Image
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from ultralytics import YOLO
from .sort import Sort
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # <- comes with tf2_ros package
from tf_transformations import quaternion_from_euler
from collections import deque
from rclpy.duration import Duration
from tf2_ros import TransformException
import time
from dataclasses import dataclass
from rclpy.time import Time

@dataclass
class AlphaBeta:
    """1-D α-β tracker for slowly-moving targets."""
    alpha: float = 0.25          # smoothing gain
    beta:  float = 0.25**2 / 2   # derived velocity gain

    x: float = None              # filtered position (m)
    v: float = 0.0               # filtered velocity (m/s)
    t: float = None              # last update timestamp (s)

    def reset(self):
        self.x, self.v, self.t = None, 0.0, None

    def update(self, z: float, t_now: float = None) -> float:
        """
        Update filter with new distance measurement *z* (metres).
        Returns current filtered distance (metres).
        """
        if t_now is None:
            t_now = time.time()

        # initialise on first sample
        if self.x is None or abs(z - self.x) > 2.0:   # discard >2 m jump
            self.x, self.v, self.t = z, 0.0, t_now
            return self.x

        dt = max(t_now - self.t, 1e-3)

        # 1 · predict
        x_pred = self.x + self.v * dt
        v_pred = self.v

        # 2 · residual
        r = z - x_pred

        # 3 · correct
        self.x = x_pred + self.alpha * r
        self.v = v_pred + self.beta  * r / dt
        self.t = t_now

        return self.x
    
class PositionFilter:
    """Filter for smoothing position measurements over time"""
    def __init__(self, window_size=5, min_confidence=3):
        self.x_buffer = deque(maxlen=window_size)
        self.y_buffer = deque(maxlen=window_size)
        self.heading_buffer = deque(maxlen=window_size)
        self.distance_buffer = deque(maxlen=window_size)
        self.window_size = window_size
        self.min_confidence = min_confidence  # Minimum measurements needed for valid output
        self.valid_measurements = 0
        self.last_valid_x = None
        self.last_valid_y = None
        self.last_valid_heading = None
        self.last_valid_distance = None

    def update(self, x, y, heading, distance):
        """Add new measurements to the filter"""
        # Check if this is a valid measurement
        if not (math.isnan(x) or math.isnan(y) or math.isnan(heading) or math.isnan(distance)):
            self.x_buffer.append(x)
            self.y_buffer.append(y)
            self.heading_buffer.append(heading)
            self.distance_buffer.append(distance)
            self.valid_measurements += 1
            
            # Update last valid values
            self.last_valid_x = x
            self.last_valid_y = y
            self.last_valid_heading = heading
            self.last_valid_distance = distance
        
        # Reset valid count if we have a full buffer of measurements
        if len(self.x_buffer) >= self.window_size:
            self.valid_measurements = self.window_size

    def get_filtered_position(self):
        """Get the filtered position if we have enough confidence"""
        if self.valid_measurements >= self.min_confidence:
            # Use median filtering to reject outliers
            x = np.median(self.x_buffer)
            y = np.median(self.y_buffer)
            heading = np.median(self.heading_buffer)
            distance = np.median(self.distance_buffer)
            return x, y, heading, distance, True
        elif self.last_valid_x is not None:
            # Return last valid measurement if we don't have enough confidence
            # but have seen some valid measurements before
            return self.last_valid_x, self.last_valid_y, self.last_valid_heading, self.last_valid_distance, False
        else:
            # No valid measurements yet
            return float('nan'), float('nan'), float('nan'), float('nan'), False
    
    def reset(self):
        """Reset the filter"""
        self.x_buffer.clear()
        self.y_buffer.clear()
        self.heading_buffer.clear()
        self.distance_buffer.clear()
        self.valid_measurements = 0

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('PerceptionNode')
        self.bridge = CvBridge()
        
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detector = YOLO('yolov8n.pt', verbose=False).to('cuda:0')
        class_names = self.detector.names
        self.get_logger().info(f'YOLO classes: {class_names}')
        
        if 0 in class_names and class_names[0] == 'person':
            self.get_logger().info("Confirmed 'person' class is at index 0")
        else:
            self.get_logger().warn("Warning: 'person' class not at expected index 0")
            # Find person class if it exists
            person_idx = None
            for idx, name in class_names.items():
                if name.lower() == 'person':
                    person_idx = idx
                    break
            if person_idx is not None:
                self.get_logger().info(f"Found 'person' at index {person_idx}")
            else:
                self.get_logger().error("Error: 'person' class not found in model")
        
        self.person_class_idx = 0  # COCO dataset standard index for person
        
        # Increase min_hits to require more consecutive detections before tracking
        # Increase max_age to keep tracks alive longer when detection is lost
        self.tracker = Sort(max_age=15, min_hits=5)  # More conservative tracking
        self.lidar_data = None
        
        # Track stable target ID
        self.tracked_id = None
        self.frames_without_detection = 0
        self.max_frames_without_detection = 10  # How long to keep a track without seeing it
        
        # Position filtering
        self.position_filter = PositionFilter(window_size=7, min_confidence=3)
        self.distance_filter = AlphaBeta(alpha=0.25)          # tune α (0.15-0.30 works well)

        # Subscribers
        self.create_subscription(
            CompressedImage,
            '/oakd/rgb/image_raw/compressed',
            self.image_callback,
            10)
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        # Publishers
        self.person_loc_pub = self.create_publisher(
            PoseStamped,
            '/see/person_location',
            10)
        self.target_status_pub = self.create_publisher(
            Bool,
            '/see/target_in_frame',
            10)
        self.raw_image_vis_pub = self.create_publisher(
            Image,
            '/see/person_detector/image_raw',
            10)
        self.compressed_image_vis_pub = self.create_publisher(
            CompressedImage,
            '/see/person_detector/image_raw/compressed',
            10)

        self.distance_publisher = self.create_publisher(
            Float32,
            '/see/target_distance',
            10
        )
        self.heading_publisher = self.create_publisher(
            Float32,
            '/see/target_heading',
            10
        )
        self.target_msg = Bool()
        self.target_msg.data = False
        
        # Camera intrinsics
        self.fx = 1012.88  # Focal length x
        self.fy = 1012.88  # Focal length y 
        self.cx = 634.40   # Principal point x
        self.cy = 363.77   # Principal point y 
        
        self.get_logger().info('PerceptionNode initialized')

    def lidar_callback(self, msg: LaserScan):
        self.lidar_data = msg

    def compute_heading(self, center):
        x_norm = (center[0] - self.cx) / self.fx
        theta_cam = -math.atan(x_norm)
        
        # transform from camera to lidar offset
        dx, dy = 0.0635, 0.0381
        x_c, y_c = math.cos(theta_cam), math.sin(theta_cam)
        theta_l = math.atan2(y_c - dy, x_c - dx)
        return theta_l

    def image_callback(self, msg: CompressedImage):
        # Convert image
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        
        # Run YOLO detection
        results = self.detector(frame)[0]
        
        # Filter for person class only
        person_indices = results.boxes.cls.cpu().numpy() == self.person_class_idx
        
        if np.any(person_indices):
            # Extract only boxes for person class
            boxes = results.boxes.xyxy.cpu().numpy()[person_indices]  # (N,4)
            scores = results.boxes.conf.cpu().numpy()[person_indices].reshape(-1,1)
            
            # Log detected persons
            self.get_logger().debug(f'Detected {len(boxes)} persons')
            
            if boxes.size:
                dets = np.hstack((boxes, scores))
                tracks = self.tracker.update(dets)
                self.frames_without_detection = 0  # Reset counter when we have detections
            else:
                tracks = np.empty((0,5))
                self.frames_without_detection += 1
        else:
            self.get_logger().debug('No persons detected')
            tracks = np.empty((0,5))
            self.frames_without_detection += 1
            
        # Check if we've lost the target for too long
        if self.frames_without_detection > self.max_frames_without_detection:
            self.tracked_id = None  # Reset tracked ID
            self.position_filter.reset()  # Reset position filter
        
        # Find best track - prioritize previous tracked ID for stability
        best = None
        max_area = 0
        
        if len(tracks) > 0:
            for x1, y1, x2, y2, track_id in tracks:
                area = (x2 - x1) * (y2 - y1)
                
                # If this is our currently tracked ID, prefer it
                if self.tracked_id is not None and track_id == self.tracked_id:
                    best = (x1, y1, x2, y2, track_id)
                    break
                
                # Otherwise, use largest area as before
                if area > max_area:
                    max_area = area
                    best = (x1, y1, x2, y2, track_id)
            
            # Update tracked ID to current best
            if best is not None:
                self.tracked_id = best[4]

        if best is not None and self.lidar_data is not None:
            x1, y1, x2, y2, track_id = best
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            
            # Draw visuals
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # Compute heading and distances
            heading = self.compute_heading((center_x, center_y))
            distance = self.get_lidar_distance(heading)
            
            # Calculate position in robot frame
            if not math.isnan(distance):
                pos_x = distance * math.cos(heading)
                pos_y = distance * math.sin(heading)
            else:
                # If distance is unknown, use fallback
                fallback_distance = 2.0  # Default fallback distance in meters
                pos_x = fallback_distance * math.cos(heading)
                pos_y = fallback_distance * math.sin(heading)
                self.get_logger().warn('Using fallback distance for position calculation')
            
            # Update position filter
            self.position_filter.update(pos_x, pos_y, heading, distance)
            
            # Get filtered position
            filtered_x, filtered_y, filtered_heading, filtered_distance, has_confidence = self.position_filter.get_filtered_position()
            
            # Draw bounding box and track ID
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
            cv2.putText(frame, f'ID:{int(track_id)}', (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Create distance labels
            if math.isnan(filtered_distance):
                distance_text = 'Distance: unknown'
            else:
                distance_text = f'Distance: {filtered_distance:.2f}m'
            
            # Create bearing label
            bearing_deg = math.degrees(filtered_heading)
            bearing_text = f'Bearing: {bearing_deg:.1f}°'
            
            # Show filtered position
            position_text = f'Position: ({filtered_x:.2f}, {filtered_y:.2f})'
            
            # Position labels to the right of the box
            text_x = x2 + 10  # 10 pixels to the right of box
            
            # Add "PERSON" label at top right
            cv2.putText(frame, 'PERSON', (text_x, y1+20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # Add measurements
            cv2.putText(frame, distance_text, (text_x, y1+50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, bearing_text, (text_x, y1+80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, position_text, (text_x, y1+110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f'Confidence: {"High" if has_confidence else "Low"}', (text_x, y1+140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


            # ------------- BUILD local pose -----------------
            ps_local              = PoseStamped()
            ps_local.header.stamp = Time()                  # = 0 → use latest
            ps_local.header.frame_id = 'base_link'
            ps_local.pose.position.x = filtered_x
            ps_local.pose.position.y = filtered_y
            ps_local.pose.position.z = 0.0
            qx,qy,qz,qw = quaternion_from_euler(0, 0, filtered_heading)
            ps_local.pose.orientation.x = qx
            ps_local.pose.orientation.y = qy
            ps_local.pose.orientation.z = qz
            ps_local.pose.orientation.w = qw



            try:
                self.get_logger().info("Waiting for transform …")

                # one-shot transform, waits up to 0.2 s
                ps_global = self.tf_buffer.transform(
                    ps_local,                 # PoseStamped in base_link
                    'odom',                   # target frame
                    timeout=Duration(seconds=0.2)
                )

            except TransformException as ex:
                self.get_logger().warn(f"TF transform failed: {ex}")
                ps_global = ps_local
            self.person_loc_pub.publish(ps_global)

            self.target_msg.data = True
        else:
            self.target_msg.data = False

        # Publish status and visualization
        self.target_status_pub.publish(self.target_msg)
        raw_out_msg = self.bridge.cv2_to_imgmsg(frame)
        compressed_out_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.raw_image_vis_pub.publish(raw_out_msg)
        self.compressed_image_vis_pub.publish(compressed_out_msg)

    def get_lidar_distance(self, angle):
        try:
            # Convert to index (0.5° resolution)
            idx = int(math.degrees(angle) * 2 + 180)
            
            # Ensure index is in valid range
            if idx < 0 or idx >= len(self.lidar_data.ranges):
                self.get_logger().warn(f'Invalid lidar index: {idx}, max: {len(self.lidar_data.ranges)-1}')
                return float('nan')
            
            # Get a small window of range values around the angle
            start_idx = max(0, idx-20)
            end_idx = min(len(self.lidar_data.ranges), idx+20)
            rngs = self.lidar_data.ranges[start_idx:end_idx]
            
            # Filter out inf/nan values
            valid_ranges = [r for r in rngs if not math.isinf(r) and not math.isnan(r)]
            
            # Check if we have any valid ranges
            if not valid_ranges:
                self.get_logger().debug(f'No valid ranges at angle {math.degrees(angle):.1f}°')
                return float('nan')
            
            # Return minimum valid range
            min_distance = min(valid_ranges)
            distance_smoothed = self.distance_filter.update(min_distance+0.1)

            # Debug log the calculated distance
            self.get_logger().info(f'Lidar distance at {math.degrees(angle):.1f}°: {min_distance:.2f}m')
            self.distance_publisher.publish(Float32(data=distance_smoothed))
            self.heading_publisher.publish(Float32(data=angle))
            return min_distance
        except Exception as e:
            self.get_logger().error(f'Error calculating lidar distance: {str(e)}')
            return float('nan')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
