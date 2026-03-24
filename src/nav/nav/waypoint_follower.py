import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.goal = None

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.create_timer(0.05, self.loop)

    def goal_cb(self, msg):
        self.goal = msg.pose

    def get_pose(self):
        try:
            t = self.tfBuffer.lookup_transform('map','base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation

            yaw = math.atan2(
                2*(q.w*q.z + q.x*q.y),
                1 - 2*(q.y*q.y + q.z*q.z)
            )
            return x,y,yaw
        except:
            return None

    def loop(self):
        if self.goal is None:
            return

        pose = self.get_pose()
        if pose is None:
            return

        x,y,yaw = pose
        gx = self.goal.position.x
        gy = self.goal.position.y

        dx = gx-x
        dy = gy-y

        dist = math.sqrt(dx*dx+dy*dy)
        target = math.atan2(dy,dx)
        err = math.atan2(math.sin(target-yaw), math.cos(target-yaw))

        cmd = Twist()

        if dist < 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.5 * dist
            cmd.angular.z = 1.5 * err

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()