import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class WaypointNav(Node):
    def __init__(self):
        super().__init__('waypoint_nav')

        self.goal = None

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.loop)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.dt = 0.05

    def goal_cb(self, msg):
        self.goal = msg.pose
        print(f"GOT GOAL: {self.goal.position.x:.2f}, {self.goal.position.y:.2f}")

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

        STOP_DIST = 0.3

        if dist < STOP_DIST:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            print("STOP")
        else:
            cmd.linear.x = min(0.4, 0.8 * dist)
            cmd.angular.z = max(min(1.5 * err, 1.0), -1.0)

        self.x += cmd.linear.x * math.cos(self.yaw) * self.dt
        self.y += cmd.linear.x * math.sin(self.yaw) * self.dt
        self.yaw += cmd.angular.z * self.dt

        print(f"pose: {self.x:.2f},{self.y:.2f} | goal: {gx:.2f},{gy:.2f} | dist: {dist:.2f}")

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = WaypointNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()