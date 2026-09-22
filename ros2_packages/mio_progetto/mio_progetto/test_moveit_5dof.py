import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class MoveIt5DOFController(Node):
    def __init__(self):
        super().__init__('moveit_5dof_controller')

        # Client per il servizio di cinematica inversa nativo di MoveIt
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Action Client per inviare la traiettoria al controller
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info('In attesa dei servizi MoveIt...')
        self.ik_client.wait_for_service()
        self.traj_client.wait_for_server()
        self.get_logger().info('MoveIt pronto per cinematica 5-DOF!')

        self.joint_names = [
            'joint_1_base_rotation',
            'joint_2_shoulder',
            'joint_3_forearm',
            'joint_4_wrist',
            'joint_5_hand'
        ]

    def move_to_xyz(self, x, y, z, time_sec=3):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.ik_link_name = 'tool0'
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout = Duration(sec=1, nanosec=0)

        # Seed di partenza standard per evitare la singolarità a braccio dritto
        seed_state = JointState()
        seed_state.name = self.joint_names
        seed_state.position = [0.0, 0.35, -0.70, 0.0, 0.0]
        request.ik_request.robot_state.joint_state = seed_state

        # Target Cartesiano: solo coordinate XYZ
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.w = 1.0  # Quaternione base neutro

        request.ik_request.pose_stamped = target_pose

        self.get_logger().info(f'Richiesta IK a MoveIt per: X={x}, Y={y}, Z={z}')
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(f'MoveIt IK Fallita! Codice errore: {response.error_code.val}')
            return False

        # Estrazione delle posizioni calcolate
        robot_names = response.solution.joint_state.name
        robot_positions = response.solution.joint_state.position
        computed_angles = [robot_positions[robot_names.index(j)] for j in self.joint_names]

        self.get_logger().info(f'Soluzione 5-DOF calcolata: {[round(a, 3) for a in computed_angles]}')

        # Invio traiettoria ad arm_controller
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = computed_angles
        point.time_from_start = Duration(sec=time_sec, nanosec=0)
        goal_msg.trajectory.points.append(point)

        self.traj_client.send_goal_async(goal_msg)
        return True


def main(args=None):
    rclpy.init(args=args)
    controller = MoveIt5DOFController()

    # Coordinate target cartesiane
    controller.move_to_xyz(x=0.00, y=-0.15, z=0.30, time_sec=3)

    rclpy.spin_once(controller, timeout_sec=3.0)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()