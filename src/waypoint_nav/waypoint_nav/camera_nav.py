import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import numpy as np
from ultralytics import YOLO


class CameraNav(Node):
    def __init__(self):
        super().__init__('camera_nav')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(ModelStates, '/model_states', self.model_cb, 10)

        self.create_subscription(Image, '/camera/camera/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_raw', self.depth_cb, 10)

        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None

        self.model = YOLO("yolov8n.pt")

        self.create_timer(0.05, self.loop)

        self.goal = None
        self.goal_received = False

        self.x = None
        self.y = None
        self.yaw = 0.0

        self.hx = None
        self.hy = None
        self.hyaw = 0.0

        self.last_seen_time = self.get_clock().now()
        self.lost_timeout = 0.5

    def goal_cb(self, msg):
        self.goal = msg.pose
        self.goal_received = True
        print("goal received")

    def model_cb(self, msg):
        try:
            i = msg.name.index('go1')
            pose = msg.pose[i]
            self.x = pose.position.x
            self.y = pose.position.y

            q = pose.orientation
            self.yaw = math.atan2(
                2*(q.w*q.z + q.x*q.y),
                1 - 2*(q.y*q.y + q.z*q.z))
        except ValueError:
            pass

    def rgb_cb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def detect_human(self):
        if self.rgb is None or self.depth is None:
            return None

        results = self.model(self.rgb, imgsz=320, verbose=False)[0]
        print("YOLO boxes:", len(results.boxes))

        for box in results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])

            if cls == 0 and conf > 0.3:
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                cx = int((x1 + x2) / 2)
                cy = max(0, y2 - 5)

                patch = self.depth[max(0,cy-4):cy+5, max(0,cx-4):cx+5]
                patch = patch[np.isfinite(patch)]

                if len(patch) == 0:
                    continue

                z = float(np.median(patch))

                if z <= 0 or z > 3.0:
                    continue

                fx = 300.0
                cx0 = self.rgb.shape[1] / 2

                X = (cx - cx0) * z / fx

                forward = z
                lateral = -X

                hx = self.x + math.cos(self.yaw) * forward - math.sin(self.yaw) * lateral
                hy = self.y + math.sin(self.yaw) * forward + math.cos(self.yaw) * lateral

                if self.hx is not None:
                    old_dist = math.sqrt((self.hx - self.x)**2 + (self.hy - self.y)**2)
                    new_dist = math.sqrt((hx - self.x)**2 + (hy - self.y)**2)

                    if new_dist > old_dist + 0.5:
                        print("DEPTH JUMP → IGNORE")
                        return None

                    if new_dist > old_dist:
                        print("DIST INCREASE → IGNORE")
                        return None

                self.hyaw = math.atan2(hy - self.y, hx - self.x)

                print(f"z={z:.2f} bx={forward:.2f} by={lateral:.2f}")

                return hx, hy

        return None

    def loop(self):
        if self.x is None or self.y is None or not self.goal_received:
            return

        human = self.detect_human()
        now = self.get_clock().now()

        if human is not None:
            new_hx, new_hy = human

            if self.hx is not None:
                self.hx = 0.8*self.hx + 0.2*new_hx
                self.hy = 0.8*self.hy + 0.2*new_hy
            else:
                self.hx, self.hy = new_hx, new_hy

            self.last_seen_time = now

        else:
            dt = (now - self.last_seen_time).nanoseconds * 1e-9

            if dt > self.lost_timeout:
                print("LOST HUMAN → STOP")
                self.hx = None
                self.hy = None

        if self.hx is None:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        safe_dist = 0.28

        dx = self.hx - self.x
        dy = self.hy - self.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist > 1e-6:
            ux = dx / dist
            uy = dy / dist
        else:
            ux, uy = 0.0, 0.0

        gx = self.hx - safe_dist * ux
        gy = self.hy - safe_dist * uy

        goal_yaw = math.atan2(self.hy - gy, self.hx - gx)

        dx = gx - self.x
        dy = gy - self.y
        rho = math.sqrt(dx*dx + dy*dy)

        theta = math.atan2(dy, dx)
        alpha = math.atan2(math.sin(theta - self.yaw),
                           math.cos(theta - self.yaw))

        beta = math.atan2(math.sin(goal_yaw - self.yaw),
                          math.cos(goal_yaw - self.yaw))

        k_rho = 0.8
        k_alpha = 1.5
        k_beta = -0.5

        if abs(alpha) > 0.3:
            vx = 0.0
            wz = 1.5 * alpha
        else:
            vx = k_rho * rho
            wz = k_alpha * alpha + k_beta * beta

        vx = min(vx, 0.6)
        wz = max(min(wz, 1.0), -1.0)

        if rho < 0.6:
            vx *= rho / 0.6

        if rho < 0.2:
            vx = min(vx, 0.15)

        stop_dist = 0.28
        align_thresh = 0.03

        if rho < stop_dist:
            vx = 0.0

            yaw_err = math.atan2(math.sin(goal_yaw - self.yaw),
                                 math.cos(goal_yaw - self.yaw))

            if abs(yaw_err) > align_thresh:
                wz = max(min(1.5 * yaw_err, 1.0), -1.0)
            else:
                wz = 0.0
                print("STOP")

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

        print(
            f"rgb={self.rgb is not None} depth={self.depth is not None}\n"
            f"robot: {self.x:.2f},{self.y:.2f} "
            f"(yaw={self.yaw:.2f}, {self.yaw*180/math.pi:.1f}deg) | "
            f"human: {self.hx if self.hx else 0:.2f},{self.hy if self.hy else 0:.2f} "
            f"(yaw={self.hyaw:.2f}, {self.hyaw*180/math.pi:.1f}deg) | "
        )


def main():
    rclpy.init()
    node = CameraNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()