import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
import serial

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.ser = serial.Serial('/dev/ttyACM0')

    def timer_callback(self):
        latest_line = None

        # Read all available data from the serial port
        while self.ser.in_waiting > 0:
            latest_line = self.ser.readline().decode('utf-8').strip()

        # If we got some data, publish it
        if latest_line:
            msg = Odometry()
            twi = list(map(float, latest_line.strip('"').split()))
            msg.twist.twist.linear.x = twi[0]
            msg.twist.twist.angular.z = twi[1]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published Odom: {msg}')


def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdometryPublisher()

    rclpy.spin(odom_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
