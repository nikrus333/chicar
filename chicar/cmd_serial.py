import rclpy
from rclpy.node import Node
import serial
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
class SerialUSB():
    def __init__(self):
        self.ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                )
    def write_data(self, data):
        data_str = str(data[0]) + ',' + str(data[1])
        data_str_bytea = data_str.encode('utf-8')
        self.ser.write(data_str_bytea)
    
    def read_data(self):
        try:
            data_str = self.ser.readline().decode('utf-8')
            print('data_str',data_str) 
            vel_str, angl_str = data_str.split(',')
        except:
            vel_str, angl_str = ['0.00', '0.00']
        return [float(vel_str), float(angl_str)]

class MinimalSubscriber(Node):
    def __init__(self):
        self.serial_1 = SerialUSB()
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            1)
        self.joints_states_pub = self.create_publisher(
            JointState,
            "/joint_states", 
            1)

        self.test_pub = self.create_publisher(
            Twist,
            "/cmd_vel", 
            1)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription  # prevent unused variable warning

        self.data_angle = 0
        self.msg = JointState()
        self.msg.name.append("vel")
        self.msg.name.append("angle")
        self.msg.velocity.append(0)
        self.msg.velocity.append(0)

        self.i = 0.0

    def listener_callback(self, msg):
        data = [msg.linear.x, msg.angular.z]
        self.data_angle = msg.angular.z
        #self.get_logger().info('Publishing: "%s"' % data)
        self.serial_1.write_data(data)
    
    def timer_callback(self):
        get_vel, get_angl = self.serial_1.read_data()
        #chicar have servo privod but dont have enocder(
        self.test_pub_cmd()
        self.msg.velocity[0] = get_vel
        self.msg.velocity[1] = math.radians(get_angl)
        self.joints_states_pub.publish(self.msg)
        #self.get_logger().info('Publishing: "%s"' % self.msg)
    
    def test_pub_cmd(self):
        # if self.i == 23:
        #     self.i = -10.0
        # self.i += 1
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        #self.test_pub.publish(msg)
        

def main(args=None):
    
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
