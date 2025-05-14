# serial_subscriber.py

import rclpy
from rclpy.node import Node                        # ROS 2 Node API :contentReference[oaicite:5]{index=5}
from geometry_msgs.msg import Twist
import serial                                      # pySerial I/O :contentReference[oaicite:6]{index=6}
from std_msgs.msg import String

class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_subscriber')
        # Open the serial port at 115200 baud, 100 ms timeout
        self.arduino_connected = 0
        try:
            # self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0)
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0)
            # Timer: poll Arduino every 0.5 s
            self.create_timer(0.001, self.poll_arduino)  # create_timer(period, callback) :contentReference[oaicite:7]{index=7}
            self.arduino_connected = 1
            self.get_logger().error('Arduino Connected, Waiting for Commands')
            self.create_timer(0.001, self.read_serial)  # create_timer(period, callback) :contentReference[oaicite:7]{index=7}

        except:
            self.get_logger().error('Arduino Not Connected!!!')
        finally:
            self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                10  # QoS history depth
            )

    def read_serial(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('ULT:'):
                    msg = String()
                    msg.data = line
                    self.get_logger().info(f'Published: {line}')
            except Exception as e:
                self.get_logger().error(f'Error reading serial data: {e}')

    def poll_arduino(self):
        """Send 'READ13' to Arduino and print 'yay' if pin 13 is HIGH."""
        try:
            # Read first line (e.g. "US1: 123")
            line1 = self.ser.readline().decode('utf-8', errors='ignore').strip()
            # if line1:
            #     self.get_logger().info(f"Line1 → {line1}")

            # Read next line (e.g. "US2: 456")
            line2 = self.ser.readline().decode('utf-8', errors='ignore').strip()
            # if line2:
            #     self.get_logger().info(f"Line2 → {line2}")
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')

    def cmd_vel_callback(self, msg):
        # Process the received Twist message
        linear = msg.linear
        angular = msg.angular
        self.get_logger().info(
            f"Received /cmd_vel -> Linear: x={linear.x}, y={linear.y}, z={linear.z}; "
            f"Angular: x={angular.x}, y={angular.y}, z={angular.z}"
        )
        if self.arduino_connected == 1:
            # # Example: Send linear.x and angular.z to Arduino
            command = f"VEL:{linear.x:.2f},{angular.z:.2f}\n"
            self.ser.write(command.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)                          # initialize ROS 2 :contentReference[oaicite:8]{index=8}
    node = SerialSubscriber()
    rclpy.spin(node)                                # enter ROS 2 loop :contentReference[oaicite:9]{index=9}
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
