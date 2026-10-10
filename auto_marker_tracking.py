
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge

import cv2
import numpy as np


class AutoMarkerTracking(Node):

    def __init__(self):
        super().__init__('auto_marker_tracking')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/satellite_arm/camera/image',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64,
            '/satellite_arm/joint2/cmd_pos',
            10
        )

        self.joint1_publisher = self.create_publisher(
            Float64,
            '/satellite_arm/joint1/cmd_pos',
            10
        )

        # Start from the last manually commanded position
        self.joint1_angle = 0.130
        self.joint2_angle = 0.595

        self.min_angle = 0.0
        self.max_angle = 0.85

        self.tolerance = 12
        self.stop_tolerance = 6
        self.horizontal_tracking = False
        self.vertical_tracking = False
        self.step_size = 0.005

        # Limit movement to one command every 0.5 seconds
        self.last_command_time = self.get_clock().now()

        self.get_logger().info(
            'Automatic marker tracking initialized'
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8'
        )

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 100, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            self.get_logger().warn(
                'Marker lost - no new movement command'
            )
            return

        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < 50:
            return

        (x, y), radius = cv2.minEnclosingCircle(largest)

        height, width = frame.shape[:2]

        target_x = width // 2
        target_y = height // 2

        error_x = x - target_x
        error_y = y - target_y

        now = self.get_clock().now()
        elapsed = (now - self.last_command_time).nanoseconds / 1e9

        if elapsed < 2.0:
            return

        self.last_command_time = now


        # Horizontal tracking using Joint 1
        if not self.horizontal_tracking and abs(error_x) > self.tolerance:
            self.horizontal_tracking = True

        if self.horizontal_tracking and abs(error_x) <= self.stop_tolerance:
            self.horizontal_tracking = False

        if self.horizontal_tracking:
            if error_x > 0:
                new_joint1 = self.joint1_angle - self.step_size
            else:
                new_joint1 = self.joint1_angle + self.step_size

            new_joint1 = max(-0.10, min(0.30, new_joint1))

            if new_joint1 != self.joint1_angle:
                command1 = Float64()
                command1.data = float(new_joint1)
                self.joint1_publisher.publish(command1)
                self.joint1_angle = new_joint1

        # Vertical tracking using Joint 2
        if not self.vertical_tracking and abs(error_y) > self.tolerance:
            self.vertical_tracking = True

        if self.vertical_tracking and abs(error_y) <= self.stop_tolerance:
            self.vertical_tracking = False

        if self.vertical_tracking:
            if error_y > 0:
                new_joint2 = self.joint2_angle + self.step_size
            else:
                new_joint2 = self.joint2_angle - self.step_size

            new_joint2 = max(
                self.min_angle,
                min(self.max_angle, new_joint2)
            )

            if new_joint2 != self.joint2_angle:
                command2 = Float64()
                command2.data = float(new_joint2)
                self.publisher.publish(command2)
                self.joint2_angle = new_joint2

        self.get_logger().info(
            f'Marker=({int(x)}, {int(y)}), '
            f'ErrorX={error_x:.1f}px, '
            f'ErrorY={error_y:.1f}px, '
            f'Joint1={self.joint1_angle:.3f}, '
            f'Joint2={self.joint2_angle:.3f}'
        )

        if (not self.horizontal_tracking and
                not self.vertical_tracking and
                abs(error_x) <= self.tolerance and
                abs(error_y) <= self.tolerance):
            self.get_logger().info(
                'ALIGNED: marker centered in both axes'
            )



def main():
    rclpy.init()
    node = AutoMarkerTracking()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\nTracking stopped.")
    finally:
        executor.remove_node(node)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
