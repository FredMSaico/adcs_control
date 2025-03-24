import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Int32MultiArray, Bool, Float32, Float32MultiArray
import numpy as np
import time

class AttitudeControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(Float32MultiArray, '/imu_euler', self.imu_callback, qos_profile)
        self.subscription = self.create_subscription(Imu, '/imu', self.gyro_callback, qos_profile)
        self.subscription = self.create_subscription(Bool, '/mode', self.mode_callback, qos_profile)
        self.subscription = self.create_subscription(Int32MultiArray, '/setpoint', self.setpoint_callback, qos_profile)
        self.publisher_set = self.create_publisher(Float32, 'yaw_setpoint', qos_profile)
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
        self.w_rw = 0

        #PID control param
        self.setpoint_yaw = 0
        self.setpoint_roll = 0
        self.setpoint_pitch = 0
        self.error_yaw = 0
        self.error_roll = 0
        self.error_pitch = 0
        self.error_prev_yaw = 0
        self.error_prev_roll = 0
        self.error_prev_pitch = 0
        self.pid_i_y = 0
        self.pid_i_r = 0
        self.pid_i_p = 0
        self.dt = 0
        self.time = time.time()
        self.t = 0

        # Constantes de control
        self.kp_y = 0.0009
        self.ki_y = 0.00000
        self.kd_y = 0.00455

        self.kp_r = 0
        self.ki_r = 0
        self.kd_r = 0

        self.kp_p = 0
        self.ki_p = 0
        self.kd_p = 0
    
    def gyro_callback(self, msg):
        self.GyroX = msg.angular_velocity.x
        self.GyroY = msg.angular_velocity.y
        self.GyroZ = msg.angular_velocity.z

    def imu_callback(self, msg):
        #self.get_logger().info('recivied: %d' % msg.header.stamp.nanosec)
        prev_time = self.time
        self.time = time.time()
        self.dt = self.time - prev_time
        
        # Obtener los ángulos de inclinación en los ejes X, Y y Z desde los datos del IMU
        self.Yaw = msg.data[0]
        self.Roll = msg.data[1]
        self.Pitch = msg.data[2]
        
        #Control
        if self.control_mode is True:
            self.t = self.t + self.dt
            self.cal_error_yaw()
            self.cal_error_roll()
            self.cal_error_pitch()
            self.cal_pid_yaw()
            self.cal_pid_roll()
            self.cal_pid_pitch()
            self.error_prev_yaw = self.error_yaw
            self.error_prev_roll = self.error_roll
            self.error_prev_pitch = self.error_pitch
        #print("PID Yaw:", self.u_yaw)
        #print("PID Roll:", self.u_roll)
        #print("PID Pitch:", self.u_pitch)
            self.w_rw = self.w_rw + (self.u_yaw/0.0037)*self.dt
            vol_set = ((((9.55*self.w_rw) - 20)/(4555-20))*4.9) + 0.1
            pwm = (1024 * vol_set)/5
            pwm = max(min(pwm, 1024), -1024)

        # Asignar los valores PWM dentro de un rango adecuado (por ejemplo, -1000 a 1000)
            self.pwm1 = 0 #int(abs(self.u_roll))
            self.pwm2 = int(abs(pwm))
            self.pwm3 = 0 #int(abs(self.u_pitch))
            #print(pwm)
            print(f"{self.Yaw:<10} {self.Pitch:<10} {self.Roll:<10} {self.GyroX:<10.2f} {self.GyroY:<10.2f} {self.GyroZ:<10.2f} {self.u_yaw:<10.1f} {self.t:<10.2f} {self.setpoint_yaw:<10.2f}")

            self.publish_topic()

    def cal_error_yaw(self):
        umbral = 180  # Umbral para el cambio de direccion
        if self.setpoint_yaw <= 180:
            if abs(self.setpoint_yaw - self.Yaw) >= umbral:
                self.error_yaw = self.setpoint_yaw - self.Yaw + 360
            else:
                self.error_yaw = self.setpoint_yaw - self.Yaw
        else:
            if abs(self.setpoint_yaw - self.Yaw) >= umbral:
                self.error_yaw = self.setpoint_yaw - self.Yaw - 360
            else:
                self.error_yaw = self.setpoint_yaw - self.Yaw
        

    def cal_error_roll(self):
        self.error_roll = self.setpoint_roll - self.Roll

    def cal_error_pitch(self):
        self.error_pitch = self.setpoint_pitch - self.Pitch


    def cal_pid_yaw(self):
        pid_p = self.kp_y*self.error_yaw                                  #Proporcional
        pid_d = self.kd_y*(self.error_yaw-self.error_prev_yaw)            #Derivativo
        self.pid_i_y = self.pid_i_y + self.ki_y*self.error_yaw            #Integral

        self.u_yaw = pid_p + pid_d + self.pid_i_y

        if self.u_yaw > 0:
            self.dir2 = True
        else:
            self.dir2 = False

    def cal_pid_roll(self):
        pid_p = self.kp_r*self.error_roll                                          #Proporcional
        pid_d = self.kd_r*(self.error_roll-self.error_prev_roll)              #Derivativo
        self.pid_i_r = self.pid_i_r + self.ki_r*self.error_roll                                  #Integral

        self.u_roll = pid_p + pid_d + self.pid_i_r
        self.u_roll = max(min(self.u_roll, 1024), -1024)
        if self.u_roll < 0:
            self.dir1 = False
        else:
            self.dir1 = True
    
    def cal_pid_pitch(self):
        pid_p = self.kp_p*self.error_pitch                                          #Proporcional
        pid_d = self.kd_p*(self.error_pitch-self.error_prev_pitch)              #Derivativo
        self.pid_i_p = self.pid_i_p + self.ki_p*self.error_pitch                                  #Integral

        self.u_pitch = pid_p + pid_d + self.pid_i_p
        self.u_pitch = max(min(self.u_pitch, 1024), -1024)
        if self.u_pitch < 0:
            self.dir3 = True
        else:
            self.dir3 = False
        
    def mode_callback(self, msg):
        self.control_mode = msg.data

    def setpoint_callback(self, msg):
        self.setpoint_yaw = msg.data[0] #180 + 20 * np.cos(0.5236*self.t) 
        self.setpoint_roll = msg.data[1]
        self.setpoint_pitch = msg.data[2]
        msg = Float32()
        #msg.data = self.setpoint_yaw
        #self.publisher_set.publish(msg)
        
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