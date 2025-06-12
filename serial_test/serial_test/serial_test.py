#!/usr/bin/env python3
import glob
import serial
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SerialSubscriber(Node):
    def __init__(self):
        super().__init__('serial_subscriber')

        # --- 1) Discover Arduino port ---
        port = None

        # Try /dev/serial/by-id symlinks first
        for path in glob.glob('/dev/serial/by-id/*'):
            if 'Arduino' in path or 'CH340' in path:
                port = path
                break

        # Fallback: scan all serial ports by VID/PID
        if port is None:
            for p in serial.tools.list_ports.comports():
                # replace with your CH340's VID/PID if different
                if (p.vid, p.pid) == (0x1a86, 0x7523) or 'Arduino' in p.description:
                    port = p.device
                    break

        if port is None:
            self.get_logger().error('❌ Could not find Arduino/CH340 port!')
            self.arduino_connected = False
        else:
            try:
                self.ser = serial.Serial(port, baudrate=115200, timeout=1.0)
                self.get_logger().info(f'✅ Connected to Arduino on {port}')
                self.arduino_connected = True
            except Exception as e:
                self.get_logger().error(f'❌ Failed to open {port}: {e}')
                self.arduino_connected = False

        # --- 2) If connected, set up polling & reading ---
        # if self.arduino_connected:
            # poll every 100 ms
            # self.create_timer(0.1, self.poll_arduino)
            # read incoming data every 100 ms
            # self.create_timer(0.1, self.read_serial)

        # --- 3) Always subscribe to /cmd_vel ---
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.send_cmd_to_arduino)

    def poll_arduino(self):
        """Periodically read two lines from the Arduino to keep the buffer flowing."""
        try:
            # you can uncomment logging below if you want to see raw lines
            # line1 = self.ser.readline().decode('utf-8', errors='ignore').strip()
            # line2 = self.ser.readline().decode('utf-8', errors='ignore').strip()
            # self.get_logger().debug(f'Polled → {line1} | {line2}')
            _ = self.ser.readline()
            _ = self.ser.readline()
        except Exception as e:
            self.get_logger().error(f'Serial poll error: {e}')

    def read_serial(self):
        """Read any available lines and log/publish those starting with ULT:."""
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('ULT:'):
                    msg = String(data=line)
                    self.get_logger().info(f'🔊 ULT message: {line}')
                    # you could publish this on a topic if desired
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """Receive /cmd_vel and forward as VEL:x,z to Arduino."""
        self.get_logger().info(
            f"Received /cmd_vel → Linear.x={msg.linear.x:.2f}, Angular.z={msg.angular.z:.2f}"
        )
        # build and write immediately
        self.latest_cmd = f"VEL:{msg.linear.x:.2f},{msg.angular.z:.2f}\n".encode('utf-8')

    def send_cmd_to_arduino(self):
            if self.arduino_connected:
                try:
                    self.ser.write(self.latest_cmd)
                    # self.ser.flush()               # force it out the USB driver ASAP
                except Exception as e:
                    self.get_logger().error(f"Write error: {e}")
                

def main(args=None):
    rclpy.init(args=args)
    node = SerialSubscriber()
    try:
        rclpy.spin(node)
    finally:
        if getattr(node, 'arduino_connected', False):
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
