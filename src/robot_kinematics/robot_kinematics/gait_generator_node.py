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
        self.motion = "stop"

        self.step_length = 0.035
        self.step_height = 0.02
        self.velocity = 0.09

        self.T = 0.6
        self.v_trot_min = 0.05
        self.v_trot_max = 0.15
        self.T_min = 0.3
        self.T_max = 0.8

        self.prev_motion = self.motion
        self.prev_gait = self.gait

        self.v_crawl_min = 0.01
        self.v_crawl_max = 0.05
        self.crawl_T_min = 0.8
        self.crawl_T_max = 2.0
        self.crawl_velocity_scale = 0.3

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

        self.allowed_motion = {
            "trot": {"forward", "backward"},
            "crawl": {"forward", "backward", "left", "right"},
        }

        self.step_length_map = {
            "crawl" : 0.035,
            "trot"  : 0.05,
        }

        self.pending_stop = False
        self.stop_request_time = None

        self.dt = 0.01
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(self.dt, self.update)
        

        self.get_logger().info("Gait Generator Node started")

    def state_callback(self, msg):
        self.robot_state = msg.data

        if msg.data == "TRANSITION_WALK_TO_STAND":
            self.pending_stand = True

    def motion_callback(self, msg):
        if msg.motion == "stop":
            self.pending_stop = True
            self.stop_request_time = self.get_clock().now().nanoseconds / 1e9
            return

        if self.pending_stop or self.pending_stand:
            return

        motion_changed = (msg.motion != self.motion)
        gait_changed = (msg.gait != self.gait)

        self.gait = msg.gait

        if msg.motion not in self.allowed_motion[self.gait]:
            self.get_logger().warn(
                f"Movimiento '{msg.motion}' no permitido para gait '{self.gait}'"
            )
            return
        
        self.step_height = msg.step_height
        self.step_length = self.step_length_map[self.gait]
        self.motion = msg.motion
        self.z0 = -msg.z_height

        if self.gait == "trot":
            self.velocity = np.clip(msg.velocity + 0.02, self.v_trot_min, self.v_trot_max + 0.05)

        elif self.gait == "crawl":
            self.velocity = np.clip(msg.velocity * self.crawl_velocity_scale + 0.01, self.v_crawl_min, self.v_crawl_max + 0.02)


        sr = self.swing_ratio()
        T_new = self.step_length / (self.velocity * (1.0 - sr))
        self.T = float(np.clip(
                T_new,
                self.crawl_T_min if self.gait == "crawl" else self.T_min,
                self.crawl_T_max if self.gait == "crawl" else self.T_max,
        ))

        if motion_changed or gait_changed:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.prev_motion = self.motion
        self.prev_gait = self.gait

    def motion_direction(self):
        if self.motion =="forward":
            return 1.0,0.0
        elif self.motion == "backward":
            return -1.0,0.0
        elif self.motion == "left":
            return 0.0,1.0
        elif self.motion == "right":
            return 0.0,-1.0
        else:
            return 0.0,0.0
        
    def swing_ratio(self):
        return 0.3 if self.gait == "crawl" else 0.4

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
            x = 0.0
            y = 0.0
            z = self.z0

            if self.robot_state == "WALK" and self.motion != "stop":
                dx, dy = self.motion_direction()
                x_step, z = self.foot_trajectory(tau)
                x = x_step * dx
                y = x_step * dy

                if not self.is_stance(tau):
                    all_in_stance = False
            

            msg = Point()
            msg.x = float(x)
            msg.y = float(self.y_offsets[leg] + y)
            msg.z = float(z)
            pub.publish(msg)

        now = self.get_clock().now().nanoseconds / 1e9

        if self.pending_stop and (
            all_in_stance or (now - self.stop_request_time) > 0.5
        ):
            self.motion = "stop"
            self.velocity = 0.0
            self.pending_stop = False
            self.stop_request_time = None
            self.start_time = now



        if self.pending_stand and all_in_stance:
            done = String()
            done.data = "WALK_TO_STAND_DONE"
            self.motion = "stop"
            self.velocity = 0.0
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
