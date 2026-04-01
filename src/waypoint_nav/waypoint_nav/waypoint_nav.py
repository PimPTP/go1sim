import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelStates
import math


class WaypointNav(Node):
    def __init__(self):
        super().__init__('waypoint_nav')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(ModelStates, '/model_states', self.model_cb, 10)

        self.create_timer(0.05, self.loop)

        self.goal = None
        self.goal_received = False

        self.x = None
        self.y = None
        self.yaw = 0.0

        self.hx = None
        self.hy = None

    def goal_cb(self, msg):
        self.goal = msg.pose
        self.goal_received = True
        print("goal received")

    def model_cb(self, msg):
        try:
            idx = msg.name.index('go1')
            pose = msg.pose[idx]
            self.x = pose.position.x
            self.y = pose.position.y
            q = pose.orientation
            self.yaw = math.atan2(
                2*(q.w*q.z + q.x*q.y),
                1 - 2*(q.y*q.y + q.z*q.z))
        except ValueError:
            pass

        try:
            idx_h = msg.name.index('human')
            pose_h = msg.pose[idx_h]
            self.hx = pose_h.position.x
            self.hy = pose_h.position.y
        except ValueError:
            self.hx = None

    def loop(self):
        if self.x is None or self.y is None or not self.goal_received:
            return

        if self.hx is not None:
            safe_dist = 0.5
            vec_x = self.hx - self.x
            vec_y = self.hy - self.y
            vec_len = math.sqrt(vec_x**2 + vec_y**2)

            if vec_len > safe_dist:
                target_dist = min(0.5, vec_len - safe_dist)
                desired_yaw = math.atan2(vec_y, vec_x)
                yaw_err = math.atan2(math.sin(desired_yaw - self.yaw), math.cos(desired_yaw - self.yaw))
                curve_factor = 0.5
                look_yaw = self.yaw + curve_factor * yaw_err

                gx = self.x + target_dist * math.cos(look_yaw)
                gy = self.y + target_dist * math.sin(look_yaw)
            else:
                gx, gy = self.x, self.y

            final_yaw = math.atan2(self.hy - self.y, self.hx - self.x)
        else:
            gx, gy = self.goal.position.x, self.goal.position.y
            final_yaw = None

        dx, dy = gx - self.x, gy - self.y
        dist = math.sqrt(dx*dx + dy*dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))

        speed_scale = max(0.2, 1 - abs(yaw_err)/math.pi)
        vx = speed_scale * 0.6 * min(dist/1.0, 1.0)
        wz = 1.2 * math.tanh(2*yaw_err)
        wz = max(min(wz, 1.0), -1.0)

        stop_dist = 0.35
        stop_yaw_thresh = 0.05
        if dist < stop_dist:
            vx = 0.0
            if final_yaw is not None:
                yaw_err2 = math.atan2(math.sin(final_yaw - self.yaw), math.cos(final_yaw - self.yaw))
                wz = max(min(1.5 * yaw_err2, 1.0), -1.0)
                if abs(yaw_err2) < stop_yaw_thresh:
                    wz = 0.0
                    print("STOP")
            else:
                wz = 0.0
                print("STOP")

        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

        print(
            f"robot: {self.x:.2f},{self.y:.2f} | "
            f"human: {self.hx:.2f},{self.hy:.2f} | "
            f"target: {gx:.2f},{gy:.2f} | dist: {dist:.2f} | yaw_err: {yaw_err:.2f}")


def main():
    rclpy.init()
    node = WaypointNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()