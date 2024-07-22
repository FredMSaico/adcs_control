import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Int32MultiArray, Bool
import numpy as np
import time
from transforms3d.euler import euler2quat

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
        self.Yaw = 0
        self.Roll = 0
        self.Pitch = 0
        self.GyroX = 0
        self.GyroY = 0
        self.GyroZ = 0

        # PID control parameters
        self.setpoint_yaw = 0
        self.setpoint_roll = 0
        self.setpoint_pitch = 0
        self.error_q = 0
        self.error_w = 0
        self.error_w_prev = 0
        self.error_prev_yaw = 0
        self.error_prev_roll = 0
        self.error_prev_pitch = 0
        self.pid_i_y = 0
        self.pid_i_r = 0
        self.pid_i_p = 0
        self.dt = 0
        self.time = time.time()
        self.t = 0

        # Control constants
        self.P = 0.0
        self.K = 0.0
        self.kd_y = 0.7
        self.Jx = 0.1  
        self.Jy = 0.1  
        self.Jz = 0.1  

    def imu_callback(self, msg):
        prev_time = self.time
        self.time = time.time()
        self.dt = self.time - prev_time
        
        # Get the inclination angles from the IMU data
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w
        self.w_x = msg.angular_velocity.x
        self.w_y = msg.angular_velocity.y
        self.w_z = msg.angular_velocity.z
        
        # Control
        if self.control_mode:
            self.t += self.dt

            self.cal_error()

            # Calculate second derivative of the desired angular velocity in z
            w_derivative = (self.setpoint_wz - self.setpoint_w_prev) / self.dt

            # Control input
            u = self.P * self.error_w + self.K * self.error_q - ((self.Jy - self.Jx) * self.w_x * self.w_y) - self.Jz * w_derivative
            
            # Integrate to get angular velocity
            w = (1 / self.Jz) * u * self.dt

            # Voltage set
            vol_set = (((w - 20) / (4555 - 20)) * 4.9) + 0.1
            pwm = (1024 * vol_set) / 5
            pwm = max(min(pwm, 1024), -1024)
            
            if pwm > 0:
                self.dir2 = True
            else:
                self.dir2 = False

            print(pwm)
            self.error_prev = self.error_q
            self.setpoint_w_prev = self.setpoint_wz

            self.pwm1 = 0
            self.pwm2 = int(abs(pwm))
            self.pwm3 = 0

            self.publish_topic()

    def cal_error(self):
        self.error_q = self.qz - self.setpoint_qz
        self.error_w = self.w_z - (self.error_q - self.error_prev) / self.dt

        # Angular velocity setpoint
        self.setpoint_wz = (self.error_q - self.error_prev) / self.dt
        
    def mode_callback(self, msg):
        self.control_mode = msg.data

    def setpoint_callback(self, msg):
        self.setpoint_yaw = msg.data[0]
        self.setpoint_roll = msg.data[1]
        self.setpoint_pitch = msg.data[2]
        
        # Convert Euler angles to quaternion
        q = euler2quat(self.setpoint_roll, self.setpoint_pitch, self.setpoint_yaw, axes='sxyz')
        self.setpoint_qz = q[2]

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