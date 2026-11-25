import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
import cv2
import mediapipe as mp
import numpy as np
import struct
import math
import time


class HandTarget(Node):

    def __init__(self):
        super().__init__('hand_target')

        self.bridge = CvBridge()

        self.create_subscription(Image, '/camera/camera/image_raw', self.image_callback, 10)
        self.create_subscription(PointCloud2, '/camera/camera/points', self.cloud_callback, 10)

        self.target_pub = self.create_publisher(PointStamped, '/hand_target', 10)

        self.cloud = None
        self.cloud_time = 0.0

        self.hands = mp.solutions.hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6)

        self.prev_xyz = None
        self.prev_time = None
        self.filtered_xyz = None
        self.last_target = None

        self.alpha = 0.8
        self.velocity_limit = 0.05
        self.target_change = 0.02


    def cloud_callback(self, msg):
        self.cloud = msg
        self.cloud_time = time.monotonic()

    def get_xyz(self, msg, px, py):

        fields = {f.name: f.offset for f in msg.fields}

        if not all(k in fields for k in ('x', 'y', 'z')):
            return None

        endian = '>' if msg.is_bigendian else '<'
        points = []

        for v in range(max(0, py - 4), min(msg.height, py + 5)):
            for u in range(max(0, px - 4), min(msg.width, px + 5)):

                offset = v * msg.row_step + u * msg.point_step

                try:
                    x = struct.unpack_from(
                        endian + 'f', msg.data,
                        offset + fields['x'])[0]

                    y = struct.unpack_from(
                        endian + 'f', msg.data,
                        offset + fields['y'])[0]

                    z = struct.unpack_from(
                        endian + 'f', msg.data,
                        offset + fields['z'])[0]

                except struct.error:
                    continue

                if all(map(math.isfinite, [x, y, z])):
                    points.append([x, y, z])

        if len(points) < 5:
            return None

        points = np.asarray(points)
        median = np.median(points, axis=0)

        keep = np.linalg.norm(points - median, axis=1) < 0.05
        points = points[keep]

        if len(points) < 3:
            return None

        if np.ptp(points[:, 2]) > 0.15:
            return None

        return np.mean(points, axis=0)

    def optical_to_base(self, p):

        return np.array([p[2] + 0.260, -p[0], -p[1] + 0.100])

    def hand_open(self, lm):

        extended = 0

        for tip, pip in [(8, 6), (12, 10), (16, 14), (20, 18)]:
            if lm[tip].y < lm[pip].y:
                extended += 1

        return extended >= 3

    def palm_up(self, points):

        v1 = points[1] - points[0]
        v2 = points[4] - points[0]

        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)

        if norm < 1e-6:
            return False

        normal = normal / norm

        return normal[2] > 0.7

    def image_callback(self, msg):

        if self.cloud is None:
            return

        if time.monotonic() - self.cloud_time > 0.3:
            return

        image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8')

        cloud = self.cloud

        h, w = image.shape[:2]

        if cloud.width != w or cloud.height != h:
            return

        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        if result.multi_hand_landmarks:

            hand = result.multi_hand_landmarks[0]

            palm_ids = [0, 5, 9, 13, 17]
            palm_points = []

            for idx in palm_ids:

                lm = hand.landmark[idx]

                px = int(lm.x * w)
                py = int(lm.y * h)

                p = self.get_xyz(cloud, px, py)

                if p is None:
                    palm_points = []
                    break

                palm_points.append(self.optical_to_base(p))

            if len(palm_points) == 5 and self.hand_open(hand.landmark):

                palm_points = np.asarray(palm_points)

                center = np.median(palm_points, axis=0)

                keep = np.linalg.norm(
                    palm_points - center, axis=1) < 0.05

                if np.count_nonzero(keep) >= 3:

                    target = np.mean(palm_points[keep], axis=0)

                    now = time.monotonic()

                    if self.filtered_xyz is None:
                        self.filtered_xyz = target
                    else:
                        self.filtered_xyz = (
                            self.alpha * target +
                            (1 - self.alpha) * self.filtered_xyz)

                    velocity_ok = True

                    if self.prev_xyz is not None:

                        dt = now - self.prev_time

                        if dt > 0.0001:

                            speed = np.linalg.norm(
                                self.filtered_xyz - self.prev_xyz) / dt

                            velocity_ok = speed < self.velocity_limit

                    self.prev_xyz = self.filtered_xyz.copy()
                    self.prev_time = now

                    changed = (
                        self.last_target is None or
                        np.linalg.norm(
                            self.filtered_xyz - self.last_target
                        ) > self.target_change)

                    if velocity_ok and changed and self.palm_up(palm_points):

                        msg_out = PointStamped()

                        msg_out.header.stamp = (
                            self.get_clock().now().to_msg())

                        msg_out.header.frame_id = 'base'

                        msg_out.point.x = float(self.filtered_xyz[0])
                        msg_out.point.y = float(self.filtered_xyz[1])
                        msg_out.point.z = float(self.filtered_xyz[2])

                        self.target_pub.publish(msg_out)

                        self.last_target = self.filtered_xyz.copy()

                        self.get_logger().info(
                            'Hand target: %.3f %.3f %.3f' %
                            (msg_out.point.x,
                             msg_out.point.y,
                             msg_out.point.z))

            mp.solutions.drawing_utils.draw_landmarks(
                image,
                hand,
                mp.solutions.hands.HAND_CONNECTIONS)

        cv2.imshow('Hand Detection', image)
        cv2.waitKey(1)

    def destroy_node(self):
        self.hands.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    rclpy.spin(HandTarget())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
