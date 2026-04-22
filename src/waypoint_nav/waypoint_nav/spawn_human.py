import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState
import os
import math

def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class SpawnHuman(Node):
    def __init__(self):
        super().__init__('spawn_human')

        self.name = "human"
        self.model_path = os.path.expanduser('~/human_model/person_standing/model.sdf')
        self.spawned = False

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')

        self.create_subscription(PoseStamped, '/goal_pose', self.move_human, 10)

        self.create_timer(1.0, self.spawn_human)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.MODEL_YAW_OFFSET = 1.5708 

    def spawn_human(self):
        if self.spawned:
            return

        if not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            return

        xml = open(self.model_path).read()

        req = SpawnEntity.Request()
        req.name = self.name
        req.xml = xml

        self.spawn_cli.call_async(req)
        self.spawned = True
        print("spawned human")

    def move_human(self, msg):
        if not self.spawned:
            return
        if not self.set_cli.wait_for_service(timeout_sec=1.0):
            return

        x_base = msg.pose.position.x
        y_base = msg.pose.position.y

        x_world = self.robot_x + math.cos(self.robot_yaw) * x_base - math.sin(self.robot_yaw) * y_base
        y_world = self.robot_y + math.sin(self.robot_yaw) * x_base + math.cos(self.robot_yaw) * y_base

        yaw = quat_to_yaw(msg.pose.orientation)
        yaw += self.MODEL_YAW_OFFSET 

        state = EntityState()
        state.name = self.name
        state.pose.position.x = x_world
        state.pose.position.y = y_world
        state.pose.position.z = msg.pose.position.z

        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)

        state.reference_frame = "world"

        req = SetEntityState.Request()
        req.state = state

        self.set_cli.call_async(req)

        print(f"move ({x_world:.2f},{y_world:.2f}) yaw={yaw:.2f}")


def main():
    rclpy.init()
    rclpy.spin(SpawnHuman())
    rclpy.shutdown()


if __name__ == '__main__':
    main()