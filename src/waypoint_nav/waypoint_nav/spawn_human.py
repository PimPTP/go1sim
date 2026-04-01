import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState
import os
import math


class SpawnHuman(Node):
    def __init__(self):
        super().__init__('spawn_human')

        self.name = "human"
        self.model_path = os.path.expanduser('~/human_model/model.sdf')
        self.spawned = False

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')

        self.create_subscription(PoseStamped, '/goal_pose', self.move_human, 10)

        self.create_timer(1.0, self.spawn_human)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

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

        state = EntityState()
        state.name = self.name
        state.pose.position.x = x_world
        state.pose.position.y = y_world
        state.pose.position.z = msg.pose.position.z
        state.pose.orientation = msg.pose.orientation
        state.reference_frame = "world"

        req = SetEntityState.Request()
        req.state = state

        self.set_cli.call_async(req)

        print(f"move ({x_world:.2f},{y_world:.2f})")


def main():
    rclpy.init()
    rclpy.spin(SpawnHuman())
    rclpy.shutdown()


if __name__ == '__main__':
    main()