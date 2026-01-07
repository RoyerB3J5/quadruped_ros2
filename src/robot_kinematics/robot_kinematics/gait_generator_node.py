import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
from robot_msgs.msg import MotionCommand
from std_msgs.msg import String

class GaitGeneratorNode(Node):
    def __init__(self):
        super().__init__('gait_generator_node')

        self.pubs = {
            'front_left':  self.create_publisher(Point, '/foot_ref_gait/front_left', 10),
            'front_right': self.create_publisher(Point, '/foot_ref_gait/front_right', 10),
            'rear_left':   self.create_publisher(Point, '/foot_ref_gait/rear_left', 10),
            'rear_right':  self.create_publisher(Point, '/foot_ref_gait/rear_right', 10),
        }

        self.done_pub = self.create_publisher(
            String, '/transition_done', 10
        )

        self.sub = self.create_subscription(
            MotionCommand, '/motion_command', self.motion_callback, 10
        )

        self.state_sub = self.create_subscription(
            String, '/robot_state', self.state_callback, 10
        )


        self.gait = "trot"
        self.pending_stand = False
        self.robot_state = "LIE"


        self.step_length = 0.035
        self.step_height = 0.01
        self.velocity = 0.0

        self.T = 0.6
        self.T_min = 0.3
        self.T_max = 1.2
        self.v_min = 0.01


        self.crawl_velocity_scale = 0.3
        self.crawl_T_min = 0.8
        self.crawl_T_max = 2.0


        self.z0 = -0.10

        self.y_offsets = {
            'front_left':  0.04005,
            'front_right': -0.04005,
            'rear_left':   0.04005,
            'rear_right':  -0.04005,
        }

        self.phase_offsets_trot = {
            'front_left':  0.0,
            'front_right': 0.5,
            'rear_left':   0.5,
            'rear_right':  0.0,
        }

        self.phase_offsets_crawl = {
            'front_left':  0.0,
            'rear_right':  0.25,
            'front_right': 0.5,
            'rear_left':   0.75,
        }

        # ------------------ Timer ------------------
        self.dt = 0.01
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(self.dt, self.update)
        

        self.get_logger().info("Gait Generator Node started")

    def state_callback(self, msg):
        self.robot_state = msg.data

        if msg.data == "TRANSITION_WALK_TO_STAND":
            self.pending_stand = True

    def motion_callback(self, msg):
        self.gait = msg.gait
        self.velocity = msg.velocity
        self.step_height = msg.step_height
        self.step_length = msg.step_length

        sr = self.swing_ratio()
        if self.velocity > self.v_min:
            T_new = self.step_length / (self.velocity * (1.0 - sr))
            self.T = float(np.clip(
                T_new,
                self.crawl_T_min if self.gait == "crawl" else self.T_min,
                self.crawl_T_max if self.gait == "crawl" else self.T_max,
            ))

        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def swing_ratio(self):
        return 0.25 if self.gait == "crawl" else 0.4

    def is_stance(self, tau):
        return tau >= self.swing_ratio()

    def foot_trajectory(self, tau):
        """
        Genera la trayectoria de un pie en coordenadas locales del cuerpo.
        tau: [0, 1] fase normalizada dentro de un ciclo de marcha
        
        Convención de coordenadas:
        - X positivo: hacia adelante (dirección de movimiento)
        - Swing: pie vuela hacia atrás (-X)
        - Stance: pie se desliza hacia atrás (-X) mientras el cuerpo avanza (+X)
        """
        sr = self.swing_ratio()
        stance_duration = 1.0 - sr
        
        body_displacement_during_stance = self.velocity * self.T * stance_duration

        if tau < sr:  

            u = tau / sr
            s = 3*u**2 - 2*u**3 
            
            x = -self.step_length/2 + self.step_length * s 
            z = self.z0 + self.step_height * np.sin(np.pi * s)
            
        else:  
            u = (tau - sr) / stance_duration
            
            x = self.step_length/2 - body_displacement_during_stance * u
            z = self.z0

        return -x, z


    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time

        phase_offsets = (
            self.phase_offsets_crawl
            if self.gait == "crawl"
            else self.phase_offsets_trot
        )

        all_in_stance = True
        
        for leg, pub in self.pubs.items():
            tau = ((t / self.T) + phase_offsets[leg]) % 1.0

            if self.robot_state == "WALK":
                x, z = self.foot_trajectory(tau)
                if not self.is_stance(tau):
                    all_in_stance = False
            else:
                x = 0.0
                z = self.z0

            msg = Point()
            msg.x = float(x)
            msg.y = float(self.y_offsets[leg])
            msg.z = float(z)
            pub.publish(msg)


        if self.pending_stand and all_in_stance:
            done = String()
            done.data = "WALK_TO_STAND_DONE"
            self.done_pub.publish(done)
            self.pending_stand = False


def main(args=None):
    rclpy.init(args=args)
    node = GaitGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
