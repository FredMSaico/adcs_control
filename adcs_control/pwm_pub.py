import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Int32, Float32, Bool, Float32MultiArray
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
import time

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('pwm_pub_node')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(Int32, '/pwm1', self.callback1, qos_profile)
        self.subscription = self.create_subscription(Int32, '/pwm2', self.callback2, qos_profile)
        self.subscription = self.create_subscription(Int32, '/pwm3', self.callback3, qos_profile)
        self.subscription = self.create_subscription(Bool, '/dir1', self.callback4, qos_profile)
        self.subscription = self.create_subscription(Bool, '/dir2', self.callback5, qos_profile)
        self.subscription = self.create_subscription(Bool, '/dir3', self.callback6, qos_profile)
        self.subscription = self.create_subscription(Int32, '/en1', self.callback7, qos_profile)
        self.subscription = self.create_subscription(Int32, '/en2', self.callback8, qos_profile)
        self.subscription = self.create_subscription(Int32, '/en3', self.callback9, qos_profile)
        self.subscription = self.create_subscription(Imu, '/imu', self.callback_imu, qos_profile)
        self.publisher_ = self.create_publisher(Int32MultiArray, 'pwm_topic', qos_profile)
        self.subscription = self.create_subscription(Float32MultiArray, '/imu_euler', self.euler_callback, qos_profile)
        timer_period = 0.01  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pwm1 = 0
        self.pwm2 = 0
        self.pwm3 = 0
        self.dir1 = True
        self.dir2 = False
        self.dir3 = True
        self.en1 = 0
        self.en2 = 1
        self.en3 = 0
        self.Yaw = 0
        self.Roll = 0
        self.Pitch = 0
        self.GyroX = 0
        self.GyroY = 0
        self.GyroZ = 0
        self.velx = 0
        self.vely = 0
        self.velz = 0
        self.pwm_range = 1024
        self.last_time = 0
        
    def callback1(self, msg):
        self.pwm1 = msg.data
        
    def callback2(self, msg):
        self.pwm2 = msg.data
        
    def callback3(self, msg):
        self.pwm3 = msg.data
        
    def callback4(self, msg):
        self.dir1 = int(msg.data)
        
    def callback5(self, msg):
        self.dir2 = int(msg.data)
        
    def callback6(self, msg):
        self.dir3 = int(msg.data)
        
    def callback7(self, msg):
        self.en1 = msg.data
        
    def callback8(self, msg):
        self.en2 = msg.data
        
    def callback9(self, msg):
        self.en3 = msg.data

    def callback_imu(self, msg):
        self.GyroX = msg.angular_velocity.x
        self.GyroY = msg.angular_velocity.y
        self.GyroZ = msg.angular_velocity.z

    def euler_callback(self, msg):
        self.Yaw = msg.data[0]
        self.Roll = msg.data[1]
        self.Pitch = msg.data[2]
        
    def timer_callback(self):
    	msg = Int32MultiArray()
    	msg.data = [self.pwm1, self.pwm2, self.pwm3, self.dir1, self.dir2, self.dir3, self.en1, self.en2, self.en3]
    	self.publisher_.publish(msg)
    	self.get_logger().info('Publishing: %d, %d, %d, %d, %d, %d, %d, %d, %d, %f' % (msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7], msg.data[8], self.velz))
    	#print(f"{self.Yaw:<10} {self.Pitch:<10} {self.Roll:<10} {self.GyroX:<10.2f} {self.GyroY:<10.2f} {self.GyroZ:<10.2f}")
        #self.get_logger().info(msg.data[5])
    	
def main(args=None):
    rclpy.init(args=args)
    pwm_pub_node = MinimalPublisher()
    rclpy.spin(pwm_pub_node)
    pwm_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
