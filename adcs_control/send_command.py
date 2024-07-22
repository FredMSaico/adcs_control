import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Bool, Float32, Float32MultiArray
import time

class AttitudeControlNode(Node):

    def __init__(self):
        super().__init__('send_command_node')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(Float32MultiArray, '/imu_euler', self.imu_callback, qos_profile)
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

    def imu_callback(self, msg):

        self.Yaw = msg.data[0]
        self.Roll = msg.data[1]
        self.Pitch = msg.data[2]
        

        # Asignar los valores PWM dentro de un rango adecuado (por ejemplo, -1000 a 1000)
        self.pwm1 = 0
        self.pwm2 = 200				# int(abs(self.u))
        self.pwm3 = 0

        # Determinar la dirección de rotación para cada motor (True para adelante, False para atrás)
        self.dir1 = False
        self.dir2 = True
        self.dir3 = True

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
