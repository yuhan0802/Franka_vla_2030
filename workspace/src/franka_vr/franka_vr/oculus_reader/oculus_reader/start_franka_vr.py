from reader import OculusReader
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Pose
from tf_transformations import (
    quaternion_from_matrix,
    quaternion_from_euler,
    quaternion_matrix,
    translation_matrix,
)
import numpy as np
from franka_msgs.action import Move,Homing, Grasp
from franka_vr.srv import SetTargetPose
class OculusPublisher(Node):
    def __init__(self):
        super().__init__('oculus_reader')
        self.oculus_reader = OculusReader()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 70.0, self.timer_callback)

        # 声明 oculus_base 的参数
        self.declare_parameter('oculus_base.x', 0.05)
        self.declare_parameter('oculus_base.y', 0.0)
        self.declare_parameter('oculus_base.z', 0.45)
        self.declare_parameter('oculus_base.roll', 3.14 / 2.0)
        self.declare_parameter('oculus_base.pitch', 0.0)
        self.declare_parameter('oculus_base.yaw', -3.14 / 2)

        # 初始化并发布静态变换
        self.publish_static_transform()

        # 初始化 TF2 Buffer 和 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('translation_scale', 0.75)
        self.declare_parameter('trigger_threshold', 0.8)
        self.declare_parameter('max_hand_delta_per_cycle', 0.05)
        self.declare_parameter('max_relative_translation', 0.30)
        self.declare_parameter('enable_orientation_control', False)
        self.declare_parameter('robot_base_frame', 'fr3_link0')
        self.declare_parameter('robot_tip_frame', 'fr3_link8')
        self.declare_parameter('gripper_action_prefix', '/fr3_gripper/franka_gripper')

        self.translation_scale = self.get_parameter('translation_scale').value
        self.trigger_threshold = self.get_parameter('trigger_threshold').value
        self.max_hand_delta_per_cycle = self.get_parameter('max_hand_delta_per_cycle').value
        self.max_relative_translation = self.get_parameter('max_relative_translation').value
        self.enable_orientation_control = self.get_parameter('enable_orientation_control').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.robot_tip_frame = self.get_parameter('robot_tip_frame').value
        gripper_action_prefix = self.get_parameter('gripper_action_prefix').value.rstrip('/')

        # Create the service client used to update MoveIt Servo's target pose.
        self.cli = self.create_client(SetTargetPose, 'set_target_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service set_target_pose...')

        # 创建 Action 客户端，用于夹爪控制
        self.action_client = ActionClient(self, Move, gripper_action_prefix + '/move')
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper move action server...')
        # 创建 Homing Action 客户端
        self.homing_client = ActionClient(self, Homing, gripper_action_prefix + '/homing')
        while not self.homing_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper homing action server...')

        # 创建 Grasp Action 客户端
        self.grasp_client = ActionClient(self, Grasp, gripper_action_prefix + '/grasp')
        while not self.grasp_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper grasp action server...')

        # 夹爪状态跟踪
        self.gripper_state = False  # None: 未初始化, True: 关闭, False: 打开
    

        self.last_gripper_send_time = self.get_clock().now()
        self.gripper_send_interval = 0.2  # 5 Hz

        # Trigger clutch state.  A new hand/robot reference pair is captured
        # on every trigger press, so changing the operator's hand pose while
        # the trigger is released cannot command an unintended robot motion.
        self.trigger_active = False
        self.hand_start_transform = None
        self.last_hand_control_transform = None
        self.robot_start_transform = None

    def publish_static_transform(self):
        """发布从 world 到 oculus_base 的静态变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'oculus_base'

        x = self.get_parameter('oculus_base.x').get_parameter_value().double_value
        y = self.get_parameter('oculus_base.y').get_parameter_value().double_value
        z = self.get_parameter('oculus_base.z').get_parameter_value().double_value
        roll = self.get_parameter('oculus_base.roll').get_parameter_value().double_value
        pitch = self.get_parameter('oculus_base.pitch').get_parameter_value().double_value
        yaw = self.get_parameter('oculus_base.yaw').get_parameter_value().double_value

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        quat = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static transform from world to oculus_base: "
                              f"position=[{x}, {y}, {z}], euler=[{roll}, {pitch}, {yaw}]")

    def publish_transform(self, transform, name):
        """发布动态变换"""
        try:
            if transform.shape != (4, 4):
                self.get_logger().warning(f"Invalid transform shape for {name}: {transform.shape}")
                return

            rotation_matrix = transform[:3, :3]
            det = np.linalg.det(rotation_matrix)
            if not np.isclose(det, 1.0, atol=1e-5):
                self.get_logger().warning(f"Invalid rotation matrix for {name}: determinant = {det}")
                return

            translation = transform[:3, 3]
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'oculus_base'
            t.child_frame_id = name
            t.transform.translation.x = float(translation[0])
            t.transform.translation.y = float(translation[1])
            t.transform.translation.z = float(translation[2])

            quat = quaternion_from_matrix(transform)
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            self.tf_broadcaster.sendTransform(t)
            # self.get_logger().info(f"Published transform for {name}")

        except Exception as e:
            self.get_logger().warning(f"Error publishing transform for {name}: {str(e)}")

    def send_target_pose(self, transform_stamped):
        """发送末端目标位姿给 set_target_pose 服务"""
        pose = Pose()
        pose.position.x = transform_stamped.transform.translation.x
        pose.position.y = transform_stamped.transform.translation.y
        pose.position.z = transform_stamped.transform.translation.z
        pose.orientation = transform_stamped.transform.rotation

        req = SetTargetPose.Request()
        req.target_pose = pose

        self.cli.call_async(req)

    @staticmethod
    def transform_to_matrix(transform):
        """Convert a Transform/TransformStamped payload into a 4x4 matrix."""
        if hasattr(transform, 'transform'):
            transform = transform.transform
        quaternion = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        translation = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ]
        return np.dot(translation_matrix(translation), quaternion_matrix(quaternion))

    def matrix_to_transform(self, matrix):
        """Build a base-frame TransformStamped from a homogeneous transform."""
        target = TransformStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.robot_base_frame
        target.child_frame_id = 'fr3_vr_target'
        target.transform.translation.x = float(matrix[0, 3])
        target.transform.translation.y = float(matrix[1, 3])
        target.transform.translation.z = float(matrix[2, 3])
        quaternion = quaternion_from_matrix(matrix)
        target.transform.rotation.x = float(quaternion[0])
        target.transform.rotation.y = float(quaternion[1])
        target.transform.rotation.z = float(quaternion[2])
        target.transform.rotation.w = float(quaternion[3])
        return target

    def begin_trigger_clutch(self):
        """Capture the current hand and robot poses when the trigger is pressed."""
        hand = self.tf_buffer.lookup_transform('world', 'oculus_right', rclpy.time.Time())
        robot = self.tf_buffer.lookup_transform(
            self.robot_base_frame, self.robot_tip_frame, rclpy.time.Time())
        self.hand_start_transform = self.transform_to_matrix(hand)
        self.last_hand_control_transform = self.hand_start_transform.copy()
        self.robot_start_transform = self.transform_to_matrix(robot)
        self.trigger_active = True
        self.get_logger().info('VR clutch engaged')

    def update_target_from_hand(self):
        """Map hand motion relative to the clutch reference into a robot target."""
        hand = self.tf_buffer.lookup_transform('world', 'oculus_right', rclpy.time.Time())
        hand_now_transform = self.transform_to_matrix(hand)
        frame_translation = np.linalg.norm(
            hand_now_transform[:3, 3] - self.last_hand_control_transform[:3, 3])
        if frame_translation > self.max_hand_delta_per_cycle:
            self.get_logger().warning(
                f'Hand tracking jump ({frame_translation:.3f} m); recalibrating VR clutch')
            self.hand_start_transform = hand_now_transform
            self.last_hand_control_transform = hand_now_transform.copy()
            self.robot_start_transform = self.transform_to_matrix(
                self.tf_buffer.lookup_transform(
                    self.robot_base_frame, self.robot_tip_frame, rclpy.time.Time()))
            return

        self.last_hand_control_transform = hand_now_transform.copy()
        # Express translation in the calibrated tracking frame, not in the
        # controller frame captured at clutch engagement.  The latter rotates
        # the translation axes with the controller and makes Cartesian motion
        # depend on how the operator happens to hold it.
        translation_delta = (
            hand_now_transform[:3, 3] - self.hand_start_transform[:3, 3]
        ) * self.translation_scale
        if np.linalg.norm(translation_delta) > self.max_relative_translation:
            self.get_logger().warning('Relative hand displacement exceeded the VR safety limit')
            return

        target = self.robot_start_transform.copy()
        target[:3, 3] += translation_delta
        if self.enable_orientation_control:
            hand_rotation_delta = np.dot(
                np.linalg.inv(self.hand_start_transform[:3, :3]),
                hand_now_transform[:3, :3],
            )
            target[:3, :3] = np.dot(
                self.robot_start_transform[:3, :3], hand_rotation_delta)
        target_transform = self.matrix_to_transform(target)
        self.tf_broadcaster.sendTransform(target_transform)
        self.send_target_pose(target_transform)

    def send_homing_goal(self):
        """发送夹爪打开命令 (Homing)"""
        goal_msg = Homing.Goal()
        self.homing_client.send_goal_async(goal_msg)
        self.get_logger().info("Sending gripper homing (open) command")
        self.gripper_state = False

    def send_grasp_goal(self):
        """发送夹爪关闭命令 (Grasp)"""
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.01    # 完全闭合
        goal_msg.speed = 0.10   # 速度 0.03 m/s
        goal_msg.force = 20.0   # 抓取力 50 N
        goal_msg.epsilon.inner = 0.1
        goal_msg.epsilon.outer = 0.2
        self.grasp_client.send_goal_async(goal_msg)
        self.get_logger().info("Sending gripper grasp (close) command")
        self.gripper_state = True

    def send_gripper_goal(self, width, action):
        """发送夹爪移动目标，不等待完成"""
        if action == 'grasp':
            self.gripper_state = True
            width = 0.01
        elif action == 'homing':
            self.gripper_state = False
            width = 0.08
        goal_msg = Move.Goal()
        goal_msg.width = width
        goal_msg.speed = 0.05 # 设置一个合理的速度（m/s）

        self.get_logger().info(f"Sending gripper goal: width={width}, speed={goal_msg.speed}")
        self.action_client.send_goal_async(goal_msg)  # 只发送，不处理反馈或结果
        

    def timer_callback(self):
        transformations, buttons = self.oculus_reader.get_transformations_and_buttons()

        # Publish the tracking pose directly.  Its axes are calibrated once
        # by the static world -> oculus_base transform above.
        if 'r' in transformations:
            self.publish_transform(transformations['r'], 'oculus_right')

        if 'l' in transformations:
            self.publish_transform(transformations['l'], 'oculus_left')

        # Use the trigger as a clutch: capture references on press, then only
        # command the robot with relative hand motion while it remains held.
        trigger_value = buttons.get('rightTrig', (0.0,))[0]
        if trigger_value >= self.trigger_threshold:
            try:
                if not self.trigger_active:
                    self.begin_trigger_clutch()
                self.update_target_from_hand()
            except Exception as e:
                self.get_logger().warning(f"Failed to update VR target: {str(e)}")
        elif self.trigger_active:
            self.trigger_active = False
            self.hand_start_transform = None
            self.last_hand_control_transform = None
            self.robot_start_transform = None
            self.get_logger().info('VR clutch released')

        # 处理 rightGrip 控制夹爪
        if 'rightGrip' in buttons:
            grip_value = buttons['rightGrip'][0]
            # 当 grip_value > 0.6 时关闭夹爪
            if grip_value > 0.6:
                if self.gripper_state == False:
                    self.send_grasp_goal()
                    # self.send_gripper_goal(0.01, 'grasp')
            # 当 grip_value < 0.4 时打开夹爪
            elif grip_value < 0.4:
                if self.gripper_state == True:
                    self.send_homing_goal()
                    # self.send_gripper_goal(0.08, 'homing')
               

        self.get_logger().debug(f'Buttons: {buttons}')

def main():
    rclpy.init()
    node = OculusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
