import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class AnalyticalArm5DOF(Node):
    def __init__(self):
        super().__init__('analytical_arm_5dof')

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info('In attesa di arm_controller...')
        self.traj_client.wait_for_server()
        self.get_logger().info('Sistema pronto!')

        self.joint_names = [
            'joint_1_base_rotation',
            'joint_2_shoulder',
            'joint_3_forearm',
            'joint_4_wrist',
            'joint_5_hand'
        ]

        # Lengths of the arm segments in meters (example values, adjust to your robot)
        self.L0 = 0.095  # heigth from base_link
        self.L1 = 0.120  # shoulder -> elbow 
        self.L2 = 0.120  # elbow -> pulse
        self.L3 = 0.120  # pulse -> End Effector (tool0)

    def solve_ik_5dof(self, x, y, z, pitch_angle_deg=-45.0):
        """
        exact geometric IK solution for a 5-DOF planar arm with a fixed pitch angle for the end effector.
        x, y, z: target position in meters
        pitch_angle_deg: angle of inclination of the end_effector (es: -45° o -90° per verticale)
        """
        pitch = math.radians(pitch_angle_deg)

        # 1. Angolo Base (Joint 1: Rotazione attorno ad Z)
        theta_1 = math.atan2(y, x)

        # Raggio orizzontale sul piano XY
        r = math.sqrt(x**2 + y**2)

        # 2. Posizione del polso (Joint 4) isolando l'orientamento della pinza
        r_wrist = r - self.L3 * math.cos(pitch)
        z_wrist = z - self.L0 - self.L3 * math.sin(pitch)

        # Distanza dalla spalla al polso
        D_sq = r_wrist**2 + z_wrist**2
        D = math.sqrt(D_sq)

        # Verifica se il punto è dentro il raggio geometrico
        if D > (self.L1 + self.L2) or D < abs(self.L1 - self.L2):
            self.get_logger().error(f'Punto ({x}, {y}, {z}) non raggiungibile geometricamente!')
            return None

        # 3. Angolo Gomito (Joint 3) tramite Teorema del Coseno (Gomito sempre alzato)
        cos_theta_3 = (D_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta_3 = max(-1.0, min(1.0, cos_theta_3))  # Clamping numerico
        theta_3 = math.acos(cos_theta_3)

        # 4. Angolo Spalla (Joint 2)
        alpha = math.atan2(z_wrist, r_wrist)
        beta = math.atan2(self.L2 * math.sin(theta_3), self.L1 + self.L2 * math.cos(theta_3))
        theta_2 = math.pi/2 - (alpha + beta)

        # 5. Angolo Polso / Inclinazione Mano (Joint 5)
        # Compensa la somma degli angoli per mantenere l'inclinazione richiesta (pitch)
        theta_5 = pitch - (alpha + beta + theta_3)

        # Il giunto 4 (rotazione assiale polso) rimane neutro per il piano
        theta_4 = 0.0

        angles = [theta_1, theta_2, theta_3, theta_4, theta_5]
        return angles

    def move_to_cartesian(self, x, y, z, pitch_deg=-30.0, time_sec=3):
        angles = self.solve_ik_5dof(x, y, z, pitch_deg)
        if angles is None:
            return False

        self.get_logger().info(f'Angoli calcolati in modo deterministico: {[round(a, 3) for a in angles]}')

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = Duration(sec=time_sec, nanosec=0)
        goal_msg.trajectory.points.append(point)

        self.traj_client.send_goal_async(goal_msg)
        return True


def main(args=None):
    rclpy.init(args=args)
    arm = AnalyticalArm5DOF()

    # Esempio: Coordinata cartesiana reale X=0.00, Y=0.15, Z=0.12 con pinza inclinata a -30 gradi
    arm.move_to_cartesian(x=0.00, y=0.15, z=0.2, pitch_deg=0.00, time_sec=3)

    rclpy.spin_once(arm, timeout_sec=3.0)
    arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()