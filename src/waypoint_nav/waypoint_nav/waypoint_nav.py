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
        self.hyaw = 0.0

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
                1 - 2*(q.y*q.y + q.z*q.z)
            )
        except ValueError:
            pass

        try:
            i = msg.name.index('human')
            pose = msg.pose[i]
            self.hx = pose.position.x
            self.hy = pose.position.y

            q = pose.orientation
            self.hyaw = math.atan2(
                2*(q.w*q.z + q.x*q.y),
                1 - 2*(q.y*q.y + q.z*q.z)
            )
        except ValueError:
            self.hx = None

    def loop(self):
        if self.x is None or self.y is None or not self.goal_received:
            return

        # TARGET
        if self.hx is not None:
            safe_dist = 0.5

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
        else:
            gx = self.goal.position.x
            gy = self.goal.position.y
            goal_yaw = None

        # ERROR
        dx = gx - self.x
        dy = gy - self.y
        rho = math.sqrt(dx*dx + dy*dy)

        theta = math.atan2(dy, dx)
        alpha = math.atan2(math.sin(theta - self.yaw),
                           math.cos(theta - self.yaw))

        if goal_yaw is not None:
            beta = math.atan2(math.sin(goal_yaw - self.yaw),
                              math.cos(goal_yaw - self.yaw))
        else:
            beta = 0.0

        # CONTROL 
        k_rho = 0.8
        k_alpha = 1.5
        k_beta = -0.5

        if abs(alpha) > 0.3:
            vx = 0.0
            wz = 1.5 * alpha
        else:
            vx = k_rho * rho
            wz = k_alpha * alpha + k_beta * beta

        # LIMIT
        vx = min(vx, 0.6)
        wz = max(min(wz, 1.0), -1.0)

        if rho < 0.5:
            vx *= rho / 0.5

        if rho < 0.2:
            vx = min(vx, 0.15)

        # FINAL ALIGN 
        stop_dist = 0.08
        align_thresh = 0.03

        if rho < stop_dist:
            vx = 0.0

            if goal_yaw is not None:
                yaw_err = math.atan2(math.sin(goal_yaw - self.yaw),
                                     math.cos(goal_yaw - self.yaw))

                if abs(yaw_err) > align_thresh:
                    wz = max(min(1.5 * yaw_err, 1.0), -1.0)
                else:
                    wz = 0.0
                    print("STOP")
            else:
                wz = 0.0
                print("STOP")

        # PUBLISH 
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

        # DEBUG 
        print(
            f"robot: {self.x:.2f},{self.y:.2f} "
            f"(yaw={self.yaw:.2f}, {self.yaw*180/math.pi:.1f}deg) | "
            f"human: {self.hx if self.hx else 0:.2f},{self.hy if self.hy else 0:.2f} "
            f"(yaw={self.hyaw:.2f}, {self.hyaw*180/math.pi:.1f}deg) | "
#            f"rho: {rho:.3f} | alpha: {alpha:.2f} | beta: {beta:.2f}"
        )


def main():
    rclpy.init()
    node = WaypointNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()