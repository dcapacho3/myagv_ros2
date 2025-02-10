#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import sys
import time

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 230400)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        self.yaw_pub = self.create_publisher(Float32, 'yaw', 10)
        self.weight1_pub = self.create_publisher(Float32, 'weight1', 10)
        self.weight2_pub = self.create_publisher(Float32, 'weight2', 10)
        
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.1,     # Small timeout for controlled reading
                write_timeout=0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            time.sleep(1)  # Allow Arduino to reset
            self.ser.reset_input_buffer()  # Clear any startup messages
            print(f"Connected to {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            sys.exit(1)
        
        self.timer = self.create_timer(0.005, self.timer_callback)  # 200Hz timer

    def timer_callback(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Print all received data
                print(line)
                
                # Check if it's a data line (contains exactly two commas)
                if line.count(',') == 2:
                    try:
                        yaw, weight1, weight2 = map(float, line.split(','))
                        
                        # Publish the data
                        self.yaw_pub.publish(Float32(data=yaw))
                        self.weight1_pub.publish(Float32(data=weight1))
                        self.weight2_pub.publish(Float32(data=weight2))
                    except ValueError:
                        pass
                        
        except (serial.SerialException, UnicodeDecodeError) as e:
            print(f"Error reading serial: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    
    try:
        rclpy.spin(serial_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        serial_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
