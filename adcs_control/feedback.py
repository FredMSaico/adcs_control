import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Int32MultiArray, Bool
import numpy as np
import time
from transforms3d.euler import euler2quat
from control import feedback_rk4, error_quaternio, boskovic_rk4

class AttitudeControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile)
        self.subscription_mode = self.create_subscription(Bool, '/mode', self.mode_callback, qos_profile)
        self.subscription_setpoint = self.create_subscription(Int32MultiArray, '/setpoint', self.setpoint_callback, qos_profile)
        self.publisher_pwm1 = self.create_publisher(Int32, '/pwm1', qos_profile)
        self.publisher_pwm2 = self.create_publisher(Int32, '/pwm2', qos_profile)
        self.publisher_pwm3 = self.create_publisher(Int32, '/pwm3', qos_profile)
        self.publisher_dir1 = self.create_publisher(Bool, '/dir1', qos_profile)
        self.publisher_dir2 = self.create_publisher(Bool, '/dir2', qos_profile)
        self.publisher_dir3 = self.create_publisher(Bool, '/dir3', qos_profile)
        self.publisher_en1 = self.create_publisher(Int32, '/en1', qos_profile)
        self.publisher_en2 = self.create_publisher(Int32, '/en2', qos_profile)
        self.publisher_en3 = self.create_publisher(Int32, '/en3', qos_profile)
        
        self.control_mode = False
        self.pwm1 = 0
        self.pwm2 = 0
        self.pwm3 = 0
        self.dir1 = False
        self.dir2 = False
        self.dir3 = True
        self.en1 = 0
        self.en2 = 0
        self.en3 = 0

        # Setpoint parameters
        self.setpoint_yaw = 0
        self.setpoint_roll = 0
        self.setpoint_pitch = 0
        self.q = np.array([[1], [0], [0], [0]])
        self.setpoint_q = np.array([[1], [0], [0], [0]])

        
        self.w_rw = np.array([[0.0], [0.0], [0.0]])
        self.dt = 0
        self.time = time.time()
        self.t = 0

        # Feedback Control constants
        self.P = np.eye(3)
        self.K = np.eye(3)
        self.I = np.array([[0.00308, 0.0, 0.0],
                           [0.0, 0.00313, 0.0],
                           [0.0, 0.0, 0.00354]])
        self.q_err_ant = np.array([[1.0], [0.0], [0.0], [0.0]])
        self.wd_ant = np.array([[0.0], [0.0], [0.0]])

        # Ganancias de control por boskovick
        self.delta = 0.01 
        self.gamma = 0.001
        self.k = 0
        self.k_ant = 0
        self.k_dot_ant = 0
        self.Umax = 0.1

    def imu_callback(self, msg):
        prev_time = self.time
        self.time = time.time()
        self.dt = self.time - prev_time

        self.q = [[msg.orientation.w], [msg.orientation.x],[msg.orientation.y], [msg.orientation.z]]
        self.w = [[msg.angular_velocity.x], [msg.angular_velocity.y], [msg.angular_velocity.z]]
        #print(f"q:{self.q}")

        #Calcula vel_angular deseada (derivada del error)
        q_err = error_quaternio(self.setpoint_q, self.q)
        q_err_dot = (q_err - self.q_err_ant) / self.dt
        self.q_err_ant = q_err
        wd = np.array([q_err_dot[1], q_err_dot[2], q_err_dot[3]]) 

        #Control FEEDBACK
        [u,wd_ante] = feedback_rk4(self.dt, self.setpoint_q, wd, self.wd_ant, self.q, self.w, self.I, self.P, self.K)
        self.wd_ant = wd_ante

        #Control BOSKOVIC
        #[u, k_ant, k_dot_ant] = boskovic_rk4(self.dt, self.setpoint_q, wd, self.wd_ant, self.q, self.w, self.delta, self.gamma, self.k, self.k_ant, self.k_dot_ant, self.Umax)
        #self.k = k_ant
        #self.k_dot = k_dot_ant

        # VERIFICA MODO DE INTERFAZ
        if self.control_mode:
            self.t += self.dt

            # Velocidad angular (RAD -> PWM)
            self.w_rw = self.w_rw + np.dot(np.linalg.inv(self.I), u) * self.dt
            vol_set = ((((9.55*self.w_rw) - 20)/(4555-20))*4.9) + 0.1
            pwm =(1024 * vol_set)/5
            pwm = np.clip(pwm, -1024, 1024)
            print(f"PWM:{pwm}")

            if pwm[2] > 0:
                self.dir2 = False
            else:
                self.dir2 = True

            self.pwm1 = 0
            self.pwm2 = int(abs(pwm[2]))
            self.pwm3 = 0

            self.publish_topic()
        
    def mode_callback(self, msg):
        self.control_mode = msg.data

    def setpoint_callback(self, msg):
        self.setpoint_yaw = msg.data[0]
        self.setpoint_roll = msg.data[1]
        self.setpoint_pitch = msg.data[2]
        
        # Convertir grados a radianes
        roll_rad = np.radians(self.setpoint_roll)
        pitch_rad = np.radians(self.setpoint_pitch)
        yaw_rad = np.radians(self.setpoint_yaw)
        
        # Convertir ángulos de Euler a cuaternión
        q = euler2quat(roll_rad, pitch_rad, yaw_rad, axes='sxyz')
        self.setpoint_q = np.array([[q[0]], [q[1]], [q[2]], [q[3]]]) 

    def publish_topic(self):    	
        self.en1 = 1
        self.en2 = 1
        self.en3 = 1
        pwm_msg1 = Int32()
        pwm_msg1.data = self.pwm1
        pwm_msg2 = Int32()
        pwm_msg2.data = self.pwm2
        pwm_msg3 = Int32()
        pwm_msg3.data = self.pwm3
        dir_msg1 = Bool()
        dir_msg1.data = self.dir1
        dir_msg2 = Bool()
        dir_msg2.data = self.dir2     
        en_msg1 = Int32()
        en_msg1.data = self.en1        	
        en_msg2 = Int32()
        en_msg2.data = self.en2
        en_msg3 = Int32()
        en_msg3.data = self.en3   	
        dir_msg3 = Bool()
        dir_msg3.data = self.dir3
        self.publisher_pwm1.publish(pwm_msg1)
        self.publisher_pwm2.publish(pwm_msg2)
        self.publisher_pwm3.publish(pwm_msg3)  
        self.publisher_dir1.publish(dir_msg1)
        self.publisher_dir2.publish(dir_msg2)
        self.publisher_dir3.publish(dir_msg3) 
        self.publisher_en1.publish(en_msg1) 
        self.publisher_en2.publish(en_msg2)  
        self.publisher_en3.publish(en_msg3)

def main(args=None):
    rclpy.init(args=args)
    control_node = AttitudeControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()