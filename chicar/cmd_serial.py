import rclpy
from rclpy.node import Node
import serial
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SerialUSB():
    def __init__(self):
        self.ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200)
    
    def write_data(self, data):
        data_str = str(data[0]) + ',' + str(data[1])
        data_str_bytea = data_str.encode('utf-8')
        self.ser.write(data_str_bytea)
    

    def read_data(self):
        data_str = self.ser.read(10).decode('utf-8')
        return data_str

class MinimalSubscriber(Node):

    def __init__(self):
        self.serial_1 = SerialUSB()
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(Twist, '/odom', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        data = [msg.linear.x, msg.angular.z]
        print(data)
        self.data_angle = msg.angular.z
        self.serial_1.write_data(data)
    
    def timer_callback(self):
        msg = Twist()
        
        data_speed = 1
        data = self.convert_odom([data_speed, self.data_angle])
        msg.linear.x = data[0]
        msg.angular.z = data[1]
        self.publisher_.publish(msg) 
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1
    
    def convert_odom(self, data):
        self.scale = 1
        self.pi = math.pi
        self.pi = math.pi
        self.len_base_car = 0.17   # lenght auto_car
        self.len_mass_c = 0.08 
        self.radius_whell = 0.0325 
        self.len_laser = 0.091  #delite
        n1 = 10
        n2 = 24
        w1 = 2 * math.pi *  data.velocity[0] * 0.229 / 60
        w2 = w1 * n1 / n2
        self.vel[0] = self.radius_whell * w2   # m/sec
        # print(data.velocity[0])
        self.vel[1] = data.velocity[1]
        flag_wheel_direction = True 
        self.current_time = rclpy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        L = self.vel[0] * dt
        alfa = 0
        if self.pos[1] != 0:
            alfa = (self.pos[1] - 1 * self.pi) * 180 / self.pi
        if alfa < 0:
            alfa = 0
        #print('alfa', alfa)
        tetta = (self.angle_whell[int(alfa)][0] - self.angle_whell[int(alfa)][1]) / 2 
        #print('tetta', tetta)
        tetta = tetta * self.pi / 180 * -1
        #
        if tetta !=0: 
            R = math.sqrt(self.len_mass_c**2 + (1 / math.tan(tetta))**2 * self.len_base_car**2)
            delta_th =  L / R
            if tetta > 0:
                delta_th *= -1
        else:
            delta_th = 0
        delta_x = L * math.cos(self.th)
        delta_y = L * math.sin(self.th) 
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

    




def main(args=None):
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
