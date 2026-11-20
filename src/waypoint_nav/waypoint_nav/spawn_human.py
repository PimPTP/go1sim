import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from gazebo_msgs.msg import EntityState, ModelStates
import os
import math
import threading
import time


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def set_yaw(orientation, yaw):
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(yaw / 2.0)
    orientation.w = math.cos(yaw / 2.0)

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SpawnHuman(Node):
    def __init__(self):
        super().__init__('spawn_human')

        self.human_name = 'human'
        self.hand_name = 'visual_hand'
        self.robot_name = 'go1'

        self.human_model_path = os.path.expanduser('~/go1_ws/human_model/person_standing/model.sdf')
        self.hand_model_path = os.path.expanduser('~/go1_ws/human_model/hand/model.sdf')

        self.spawn_step = 0
        self.spawn_future = None

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')

        self.create_subscription(PoseStamped, '/goal_pose', self.move_human, 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, qos_profile_sensor_data)

        self.hand_pub = self.create_publisher(PoseStamped, '/hand_target_pose', 10)

        self.human_x = 0.0
        self.human_y = 0.0
        self.human_z = 0.0
        self.human_yaw = 0.0
        self.human_pose_received = False
        self.pose_lock = threading.Lock()

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_received = False

        self.last_hx = 0.35
        self.last_hy = 0.0
        self.last_hz = 0.8
        self.hand_yaw_offset = 0.0
        self.offset_lock = threading.Lock()

        self.hand_update_pending = False
        self.human_move_pending = False

        self.MODEL_YAW_OFFSET = math.pi / 2.0

        self.create_timer(1.0, self.spawn_models)
        self.create_timer(0.1, self.update_hand_pose)

        self.terminal_thread = threading.Thread(
            target=self.terminal_input_loop,
            daemon=True)
        self.terminal_thread.start()

        print('[INFO] SpawnHuman started.')
        print('Commands:')
        print('  1 x y   : move human (world position, metres)')
        print('  1 yaw   : rotate human (radians)')
        print('  2 x y z : set hand offset relative to human (metres)')
        print('  2 yaw   : rotate hand relative to human (radians)')


    def spawn_models(self):
        if self.spawn_step >= 2:
            return

        if self.spawn_future is not None:
            if not self.spawn_future.done():
                return

            try:
                result = self.spawn_future.result()
                if not result.success:
                    print('[ERROR] Spawn failed:', result.status_message)
                    self.spawn_future = None
                    return

                print('[INFO]', 'Human' if self.spawn_step == 0
                      else 'Visual hand', 'spawned.')

                if self.spawn_step == 0:
                    with self.pose_lock:
                        self.human_x = 0.0
                        self.human_y = 0.0
                        self.human_z = 0.0
                        self.human_yaw = 0.0
                        self.human_pose_received = True

                self.spawn_step += 1
                self.spawn_future = None

            except Exception as e:
                print('[ERROR] Spawn failed:', e)
                self.spawn_future = None
                return

        if not self.spawn_cli.service_is_ready():
            return

        if self.spawn_step == 0:
            model_path = self.human_model_path
            model_name = self.human_name
        else:
            model_path = self.hand_model_path
            model_name = self.hand_name

        if not os.path.isfile(model_path):
            print('[ERROR] Model not found:', model_path)
            return

        try:
            with open(model_path, 'r') as f:
                xml = f.read()
        except OSError as e:
            print('[ERROR] Cannot read model:', e)
            return

        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = xml

        if self.spawn_step == 0:
            set_yaw(req.initial_pose.orientation, self.MODEL_YAW_OFFSET)

        self.spawn_future = self.spawn_cli.call_async(req)

    def model_states_callback(self, msg):
        if self.human_name in msg.name:
            idx = msg.name.index(self.human_name)
            pose = msg.pose[idx]
            actual_yaw = quat_to_yaw(pose.orientation)

            with self.pose_lock:
                self.human_x = pose.position.x
                self.human_y = pose.position.y
                self.human_z = pose.position.z
                self.human_yaw = normalize_angle(
                    actual_yaw - self.MODEL_YAW_OFFSET
                )
                self.human_pose_received = True

        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            pose = msg.pose[idx]
            self.robot_x = pose.position.x
            self.robot_y = pose.position.y
            self.robot_yaw = quat_to_yaw(pose.orientation)
            self.robot_pose_received = True

    def set_human_pose(self, x, y, z, desired_yaw):
        if self.human_move_pending:
            print('[WAIT] Previous person command is still running.')
            return False

        if not self.set_cli.service_is_ready():
            print('[WAIT] /set_entity_state service is not available.')
            return False

        desired_yaw = normalize_angle(desired_yaw)

        state = EntityState()
        state.name = self.human_name
        state.reference_frame = 'world'
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        set_yaw(
            state.pose.orientation,
            desired_yaw + self.MODEL_YAW_OFFSET)

        req = SetEntityState.Request()
        req.state = state

        target_pose = (float(x), float(y), float(z), desired_yaw)
        self.human_move_pending = True
        future = self.set_cli.call_async(req)
        future.add_done_callback(
            lambda result_future: self.human_move_callback(
                result_future, target_pose))
        return True

    def human_move_callback(self, future, target_pose):
        self.human_move_pending = False
        try:
            result = future.result()
            if not result.success:
                print('[ERROR] Gazebo rejected person pose:', result.status_message)
                return

            with self.pose_lock:
                (self.human_x, self.human_y,
                 self.human_z, self.human_yaw) = target_pose
                self.human_pose_received = True

            print('[OK] Person pose updated in Gazebo.')
        except Exception as e:
            print('[ERROR] Person update failed:', e)

    def move_human(self, msg):
        if self.spawn_step < 1:
            return

        frame = msg.header.frame_id.strip().lower()
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        if frame in ('', 'world', 'map'):
            x_world, y_world = x, y
        elif frame in ('base', 'base_link', 'trunk'):
            if not self.robot_pose_received:
                print('[WAIT] Go1 pose not received.')
                return
            c = math.cos(self.robot_yaw)
            s = math.sin(self.robot_yaw)
            x_world = self.robot_x + c * x - s * y
            y_world = self.robot_y + s * x + c * y
        else:
            print('[ERROR] Unsupported frame:', frame)
            return

        desired_yaw = quat_to_yaw(msg.pose.orientation)
        self.set_human_pose(x_world, y_world, z, desired_yaw)
        print(f'[HUMAN] Goal -> ({x_world:.2f}, {y_world:.2f}, {z:.2f})')

    def calculate_hand_pose(self):
        with self.pose_lock:
            human_x = self.human_x
            human_y = self.human_y
            human_z = self.human_z
            person_yaw = self.human_yaw

        with self.offset_lock:
            x = self.last_hx
            y = self.last_hy
            z = self.last_hz
            hand_yaw = self.hand_yaw_offset

        c = math.cos(person_yaw)
        s = math.sin(person_yaw)

        hx = human_x + c * x - s * y
        hy = human_y + s * x + c * y
        hz = human_z + z

        return hx, hy, hz, normalize_angle(person_yaw + hand_yaw)

    def update_hand_pose(self):
        if self.spawn_step < 2 or not self.human_pose_received:
            return
        if self.hand_update_pending:
            return
        if not self.set_cli.service_is_ready():
            return

        hx, hy, hz, hand_yaw = self.calculate_hand_pose()

        state = EntityState()
        state.name = self.hand_name
        state.reference_frame = 'world'
        state.pose.position.x = hx
        state.pose.position.y = hy
        state.pose.position.z = hz
        set_yaw(state.pose.orientation, hand_yaw)

        req = SetEntityState.Request()
        req.state = state

        self.hand_update_pending = True
        future = self.set_cli.call_async(req)
        future.add_done_callback(self.hand_update_callback)

        hand_msg = PoseStamped()
        hand_msg.header.stamp = self.get_clock().now().to_msg()
        hand_msg.header.frame_id = 'world'
        hand_msg.pose.position.x = hx
        hand_msg.pose.position.y = hy
        hand_msg.pose.position.z = hz
        set_yaw(hand_msg.pose.orientation, hand_yaw)
        self.hand_pub.publish(hand_msg)

    def hand_update_callback(self, future):
        self.hand_update_pending = False
        try:
            result = future.result()
            if not result.success:
                print('[ERROR] Cannot update hand:', result.status_message)
        except Exception as e:
            print('[ERROR] Hand update failed:', e)

    def terminal_input_loop(self):
        while self.spawn_step < 2 and rclpy.ok():
            time.sleep(0.5)

        while rclpy.ok():
            try:
                raw = input('Command (1/2 ...): ').strip()
                if not raw:
                    continue

                values = raw.replace(',', ' ').split()
                if len(values) not in (2, 3, 4):
                    print('[ERROR] Use: 1 x y | 1 yaw | 2 x y z | 2 yaw')
                    continue

                target = int(values[0])
                if target not in (1, 2):
                    print('[ERROR] First value must be 1 (person) or 2 (hand).')
                    continue

                numbers = [float(v) for v in values[1:]]

                if (target == 1 and len(numbers) not in (1, 2)) or \
                   (target == 2 and len(numbers) not in (1, 3)):
                    print('[ERROR] Use: 1 x y | 1 yaw | 2 x y z | 2 yaw')
                    continue

                if target == 1 and len(numbers) == 2:
                    x, y = numbers
                    with self.pose_lock:
                        z = self.human_z
                        yaw = self.human_yaw
                    if self.set_human_pose(x, y, z, yaw):
                        print(f'[HUMAN] Move -> x={x:.2f}, y={y:.2f}')

                elif target == 2 and len(numbers) == 3:
                    x, y, z = numbers
                    with self.offset_lock:
                        self.last_hx = x
                        self.last_hy = y
                        self.last_hz = z
                    print(f'[HAND] Local offset -> ({x:.2f}, {y:.2f}, {z:.2f})')

                elif target == 1:
                    yaw = numbers[0]
                    with self.pose_lock:
                        x = self.human_x
                        y = self.human_y
                        z = self.human_z
                    if self.set_human_pose(x, y, z, yaw):
                        print(f'[HUMAN] Rotate -> yaw={yaw:.3f} rad')

                else:
                    yaw = numbers[0]
                    with self.offset_lock:
                        self.hand_yaw_offset = yaw
                    print(f'[HAND] Relative yaw -> {yaw:.3f} rad')

            except ValueError:
                print('[ERROR] Invalid number.')
            except EOFError:
                break
            except Exception as e:
                print('[ERROR]', e)


def main():
    rclpy.init()
    rclpy.spin(SpawnHuman())
    rclpy.shutdown()


if __name__ == '__main__':
    main()