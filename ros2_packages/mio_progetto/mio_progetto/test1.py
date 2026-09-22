import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class CartesianArmController(Node):
    def __init__(self):
        super().__init__('cartesian_arm_controller')

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Sottoscrizione per memorizzare la posizione attuale del robot
        self.current_joint_state = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('In attesa dei nodi...')
        self.ik_client.wait_for_service()
        self.traj_client.wait_for_server()
        self.get_logger().info('Sistema pronto!')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def compute_ik_and_move(self, x, y, z, time_sec=3):
        joint_names = [
            'joint_1_base_rotation',
            'joint_2_shoulder',
            'joint_3_forearm',
            'joint_4_wrist',
            'joint_5_hand'
        ]

        # Lista di Seed States da tentare in sequenza per battere i minimi locali di KDL
        candidate_seeds = []
        
        # 1. Prova prima con lo stato corrente reale del robot
        if self.current_joint_state is not None:
            candidate_seeds.append(self.current_joint_state)

        # 2. Seed per punti ravvicinati/piegati
        seed_bent = JointState()
        seed_bent.name = joint_names
        seed_bent.position = [0.0, 0.5, -0.8, 0.3, 0.0]
        candidate_seeds.append(seed_bent)

        # 3. Seed alternativo a riposo medio
        seed_mid = JointState()
        seed_mid.name = joint_names
        seed_mid.position = [0.0, 0.2, -0.2, 0.0, 0.0]
        candidate_seeds.append(seed_mid)

        response = None
        for i, seed in enumerate(candidate_seeds):
            request = GetPositionIK.Request()
            request.ik_request.group_name = 'arm'
            request.ik_request.ik_link_name = 'tool0'
            request.ik_request.avoid_collisions = False
            request.ik_request.timeout = Duration(sec=0, nanosec=500_000_000) # 0.5s per tentativo
            request.ik_request.robot_state.joint_state = seed

            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.pose.position.x = float(x)
            target_pose.pose.position.y = float(y)
            target_pose.pose.position.z = float(z)
            target_pose.pose.orientation.w = 1.0

            request.ik_request.pose_stamped = target_pose

            future = self.ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()

            if res.error_code.val == 1:
                response = res
                self.get_logger().info(f'IK risolta con tentativo seed #{i+1}!')
                break

        if response is None or response.error_code.val != 1:
            self.get_logger().error(f'IK Fallita per ({x}, {y}, {z}): punto non raggiungibile o fuori limiti giunti.')
            return False

        # Estrazione angoli
        robot_joint_names = response.solution.joint_state.name
        robot_joint_positions = response.solution.joint_state.position
        computed_positions = [robot_joint_positions[robot_joint_names.index(j)] for j in joint_names]

        self.get_logger().info(f'Angoli calcolati: {[round(p, 3) for p in computed_positions]}')

        # Invio Traiettoria
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
    controller = CartesianArmController()

    # Lascia un attimo al nodo per ricevere il primo /joint_states
    rclpy.spin_once(controller, timeout_sec=0.5)

    # Test con il punto critico
    controller.compute_ik_and_move(x=0.00, y=0.00, z=0.30, time_sec=3)

    rclpy.spin_once(controller, timeout_sec=3.0)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()