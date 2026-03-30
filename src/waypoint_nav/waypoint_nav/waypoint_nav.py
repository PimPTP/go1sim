import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelStates
import math


class WaypointNav(Node):
    def __init__(self):
        super().__init__('waypoint_nav')

        self.goal = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.reached = False

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(ModelStates, '/model_states', self.model_cb, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.loop)

    def goal_cb(self, msg):
        self.goal = msg.pose
        print(f"GOT GOAL: {self.goal.position.x:.2f}, {self.goal.position.y:.2f}")

    def model_cb(self, msg):
        try:
            idx = msg.name.index('go1')

            pose = msg.pose[idx]

            self.x = pose.position.x
            self.y = pose.position.y

            q = pose.orientation

            self.yaw = math.atan2(
                2*(q.w*q.z + q.x*q.y),
                1 - 2*(q.y*q.y + q.z*q.z)
            )

        except ValueError:
            pass

    def loop(self):
        if self.goal is None:
            return

        gx = self.goal.position.x
        gy = self.goal.position.y

        dx = gx - self.x
        dy = gy - self.y

        dist = math.sqrt(dx*dx + dy*dy)

        target = math.atan2(dy, dx)
        err = math.atan2(math.sin(target - self.yaw), math.cos(target - self.yaw))

        cmd = Twist()

        STOP_DIST = 0.25
        GO_DIST = 0.35

        if self.reached:
            if dist > GO_DIST:
                self.reached = False
        else:
            if dist < STOP_DIST:
                self.reached = True

        if self.reached:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            print("STOP")
        else:
            cmd.linear.x = min(0.4, 0.8 * dist)
            cmd.angular.z = max(min(1.5 * err, 1.0), -1.0)

        print(
            f"pose: {self.x:.2f},{self.y:.2f} | "
            f"goal: {gx:.2f},{gy:.2f} | "
            f"dist: {dist:.2f} | "
            f"vx: {cmd.linear.x:.2f} | "
            f"wz: {cmd.angular.z:.2f}"
        )

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = WaypointNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()