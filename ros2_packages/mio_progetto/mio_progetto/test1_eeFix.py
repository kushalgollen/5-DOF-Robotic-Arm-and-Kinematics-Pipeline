import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class DirectGripperArmController(Node):
    def __init__(self):
        super().__init__('direct_gripper_arm_controller')

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info('In attesa dei servizi MoveIt...')
        self.ik_client.wait_for_service()
        self.traj_client.wait_for_server()
        self.get_logger().info('Servizi pronti!')

    def compute_ik_and_move(self, x, y, z, pitch_offset=0.0, time_sec=3):
        """
        pitch_offset: 
          0.0 = pinza parallela al suolo (orizzontale)
          1.57 (pi/2) = pinza rivolta verso il basso (verticale)
        """
        joint_names = [
            'joint_1_base_rotation',
            'joint_2_shoulder',
            'joint_3_forearm',
            'joint_4_wrist',
            'joint_5_hand'
        ]

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.ik_link_name = 'hand_link'
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout = Duration(sec=1, nanosec=0)

        # Seed di appoggio per il calcolo dei link 1-4
        fixed_seed = JointState()
        fixed_seed.name = joint_names
        fixed_seed.position = [0.0, 0.35, -0.70, 0.0, 0.0]
        request.ik_request.robot_state.joint_state = fixed_seed

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.w = 1.0

        request.ik_request.pose_stamped = target_pose

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(f'IK Fallita per ({x}, {y}, {z}) con codice: {response.error_code.val}')
            return False

        robot_joint_names = response.solution.joint_state.name
        robot_joint_positions = response.solution.joint_state.position
        computed_positions = [robot_joint_positions[robot_joint_names.index(j)] for j in joint_names]

        # CONTROLLO DIRETTO: Calcolo dell'angolo del giunto 5
        # Compensazione geometrica: theta_5 = -(theta_spalla + theta_gomito) + offset
        theta_shoulder = computed_positions[1]
        theta_forearm  = computed_positions[2]
        
        computed_positions[4] = -(theta_shoulder + theta_forearm) + pitch_offset

        self.get_logger().info(f'Angoli finali (con pinza controllata): {[round(p, 3) for p in computed_positions]}')

        # Invio traiettoria
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = computed_positions
        point.time_from_start = Duration(sec=time_sec, nanosec=0)
        goal_msg.trajectory.points.append(point)

        self.traj_client.send_goal_async(goal_msg)
        return True


def main(args=None):
    rclpy.init(args=args)
    controller = DirectGripperArmController()

    # Esempio: coordinate XYZ con pinza parallela al suolo (pitch_offset = 0.0)
    controller.compute_ik_and_move(x=-0.15, y=0.00, z=0.25, pitch_offset=0.0, time_sec=3)

    rclpy.spin_once(controller, timeout_sec=3.0)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()