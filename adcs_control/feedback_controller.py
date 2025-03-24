import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Int32MultiArray, Bool, Float32MultiArray
import numpy as np
import time
from transforms3d.euler import euler2quat
from quaternionFunctions import error_quaternio, xi_operator
from bosckControl import gain_estimator_bosk, boskovic_control
from feedbackControl import control_feedback
from control import feedback_rk4, boskovic_rk4
from reference import calc_wd

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
        self.subscription = self.create_subscription(Float32MultiArray, '/imu_euler', self.euler_callback, qos_profile)
        self.subscription_mode = self.create_subscription(Bool, '/mode', self.mode_callback, qos_profile)
        self.subscription_setpoint = self.create_subscription(Float32MultiArray, '/setpoint', self.setpoint_callback, qos_profile)
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
        self.Yaw = 0
        self.Roll = 0
        self.Pitch = 0

        # Setpoint parameters
        self.setpoint_yaw = 0
        self.setpoint_roll = 0
        self.setpoint_pitch = 0
        self.q = np.array([[1], [0], [0], [0]])
        self.setpoint_q = np.array([[1], [0], [0], [0]])
        self.qd_prev = np.array([[1], [0], [0], [0]])
        self.qd_dot = np.array([[1], [0], [0], [0]])
        
        self.w_rw = np.array([[0.0], [21.0], [0.0]])
        self.dt = 0
        self.time = time.time()
        self.t = 0

        # Feedback Control constants
        self.P = np.array([[0.0007, 0.0, 0.0],
                           [0.0, 0.0007, 0.0],
                           [0.0, 0.0, 0.005]])
        self.K = np.array([[0.001, 0.0, 0.0],
                           [0.0, 0.001, 0.0],
                           [0.0, 0.0, 0.004]])
        self.I = np.array([[0.0038, 0.0, 0.0],
                           [0.0, 0.0039, 0.0],
                           [0.0, 0.0, 0.0037]])
        self.Irw = np.array([[0.000027, 0.0, 0.0],
                           [0.0, 0.000027, 0.0],
                           [0.0, 0.0, 0.000027]])
        self.q_err_ant = np.array([[1.0], [0.0], [0.0], [0.0]])
        self.wd_prev = np.array([[0.0], [0.0], [0.0]])

        # Ganancias de control por boskovick
        self.delta = 1.08
        self.gamma = 0.027
        self.k = 2
        self.k_ant = self.k
        self.k_dot_ant = 0
        self.Umax = 0.005

    def imu_callback(self, msg):
        prev_time = self.time
        self.time = time.time()
        self.dt = self.time - prev_time

        self.q = [[msg.orientation.w], [msg.orientation.x],[msg.orientation.y], [msg.orientation.z]]
        self.w = [[msg.angular_velocity.x], [msg.angular_velocity.y], [msg.angular_velocity.z]]
        X = np.array([[msg.orientation.w], [msg.orientation.x],[msg.orientation.y], [msg.orientation.z], [msg.angular_velocity.x], [msg.angular_velocity.y], [msg.angular_velocity.z]])
        #print(f"q:{self.q}")

        #Calcula vel_angular deseada (derivada del error)
        q_err = error_quaternio(self.setpoint_q, self.q)
        q_err_dot = (q_err - self.q_err_ant) / self.dt
        self.q_err_ant = q_err
        wd = np.linalg.pinv(xi_operator(self.setpoint_q.flatten())) @ (2 * self.qd_dot.flatten()) #np.array([[0.0], [0.0], [np.radians(-20*0.5236*np.sin(0.5236*self.t))]])   #np.array([[0.0], [0.0], [0.0]]) 
        wd = np.array([[wd[0]], [wd[1]], [wd[2]]])
        wd_dot =  (wd - self.wd_prev)/self.dt #np.array([[0.0], [0.0], [np.radians(-20*0.5236*0.5236*np.cos(0.5236*self.t))]])  #np.array([[0.0], [0.0], [0.0]]) 
        self.wd_prev = wd

        #Control FEEDBACK
        u = control_feedback(self.I, X, q_err, wd, wd_dot, self.P, self.K)
        [u,wd_ante] = feedback_rk4(self.dt, self.setpoint_q, wd, self.wd_ant, self.q, self.w, self.I, self.P, self.K)
        self.wd_ant = wd_ante

        # VERIFICA MODO DE INTERFAZ
        if self.control_mode:
            self.t += self.dt

            # Velocidad angular (RAD -> PWM)
            self.w_rw = self.w_rw + np.dot(np.linalg.inv(self.Irw), u) * self.dt
            vol_set = ((((9.55*self.w_rw) - 20)/(4555-20))*4.9) + 0.1
            pwm =(1024 * vol_set)/5
            pwm = np.clip(pwm, -1024, 1024)
            #print(f"PWM:{pwm}")
            print(f"{self.Yaw:<10} {self.Pitch:<10} {self.Roll:<10} {msg.angular_velocity.x:<10.2f} {msg.angular_velocity.y:<10.2f} {msg.angular_velocity.z:<10.2f} {u[2][0]:<10.5f} {self.t:<10.2f} {self.setpoint_yaw:<10.2f} {pwm[2][0]:<10.2f}")

            if pwm[0] > 0:
                self.dir3 = True
            else:
                self.dir3 = False

            if pwm[1] > 0:
                self.dir1 = False
            else:
                self.dir1 = True
            
            if pwm[2] > 0:
                self.dir2 = False
            else:
                self.dir2 = True

            self.pwm1 = int(abs(pwm[1]))
            self.pwm2 = int(abs(pwm[2]))
            self.pwm3 = int(abs(pwm[0]))

            self.publish_topic()
        
    def mode_callback(self, msg):
        self.control_mode = msg.data

    def setpoint_callback(self, msg):
        self.setpoint_yaw = 180 + 20 * np.cos(0.5236*self.t) #
        self.setpoint_roll = msg.data[1]
        self.setpoint_pitch = msg.data[2]
        
        # Convertir grados a radianes
        roll_rad = np.radians(self.setpoint_roll)
        pitch_rad = np.radians(self.setpoint_pitch)
        yaw_rad = np.radians(self.setpoint_yaw)
        
        # Convertir ángulos de Euler a cuaternión
        q = euler2quat(roll_rad, pitch_rad, yaw_rad, axes='sxyz')
        self.setpoint_q = np.array([[q[0]], [q[1]], [q[2]], [q[3]]])
        if self.dt > 0.04:
            self.qd_dot = (self.setpoint_q - self.qd_prev) / self.dt
            self.qd_prev = self.setpoint_q 

    def euler_callback(self, msg):
        self.Yaw = msg.data[0]
        self.Roll = msg.data[1]
        self.Pitch = msg.data[2]

    def publish_topic(self):    	
        self.en1 = 0
        self.en2 = 1
        self.en3 = 0
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