import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import ikpy.chain
import numpy as np

class Arm5DOFIKPyController(Node):
    def __init__(self):
        super().__init__('arm_5dof_ikpy_controller')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Percorso del file URDF
        urdf_path = os.path.expanduser(
            '/home/kgollen/ros2_ws_test1/src/robotic_arm_description_test1/urdf/ros2_Robo_Arm_Robus.urdf'
        )

        # Carica la catena cinematica dall'URDF specificando il frame di base
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["base_link"]
        )

        # Nomi esatti dei 5 giunti definiti nell'URDF
        self.joint_names = [
            'joint_1_base_rotation',
            'joint_2_shoulder',
            'joint_3_forearm',
            'joint_4_wrist',
            'joint_5_hand'
        ]

        # Maschera per abilitare esclusivamente i 5 giunti fisici reali
        # Link 0 = base_link (disabilitato), Link 1..5 = giunti motorizzati
        self.active_links_mask = [False, True, True, True, True, True]

        # Seme iniziale di default (configurazione di riposo standard)
        self.current_angles = [0.0, 0.0, 0.35, -0.70, 0.0, 0.0]

        self.get_logger().info('Nodo IKPy 5-DOF avviato e pronto.')

    def move_to_cartesian(self, x, y, z):
        target_position = [float(x), float(y), float(z)]
        self.get_logger().info(f'Calcolo IK per coordinate: X={x}, Y={y}, Z={z}')

        # Risoluzione cinematica inversa a 5 DOF per sola posizione
        ik_solution = self.chain.inverse_kinematics(
            target_position=target_position,
            initial_position=self.current_angles,
            active_links_mask=self.active_links_mask
        )

        # Aggiorna la posa memorizzata per il calcolo successivo
        self.current_angles = ik_solution

        # Estrae i 5 angoli effettivi (escludendo il link 0 della base)
        active_positions = list(ik_solution[1:6])
        self.get_logger().info(
            f'Angoli calcolati (deg): {[round(np.rad2deg(a), 2) for a in active_positions]}'
        )

        # Invio messaggio a RViz
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(p) for p in active_positions]

        self.joint_pub.publish(msg)
        return active_positions


def main(args=None):
    rclpy.init(args=args)
    controller = Arm5DOFIKPyController()

    # Esempio: invio al punto cartesiano desiderato (metri rispetto a base_link)
    controller.move_to_cartesian(x=0.00, y=0.12, z=0.12)

    # Mantieni il nodo attivo per consentire la pubblicazione del messaggio
    rclpy.spin_once(controller, timeout_sec=1.0)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()