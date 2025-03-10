#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class WheelEncoderNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder_node')
        
        # Constants
        self.HEADER = bytes([0xfe, 0xfe])
        self.WHEEL_RADIUS = 0.1  # meters - adjust to your robot's wheel size
        self.ENCODER_TICKS_PER_REVOLUTION = 4096  # adjust to your encoder's resolution
        self.WHEEL_SEPARATION_WIDTH = 0.5  # meters - adjust to your robot's width
        self.WHEEL_SEPARATION_LENGTH = 0.4  # meters - adjust to your robot's length
        
        # Initialize serial port (using same settings as original code)
        self.serial_port = serial.Serial(
            port='/dev/ttyAMA2',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        
        # Initialize wheel positions
        self.wheel_positions = {
            'left_wheel_front': 0.0,
            'right_wheel_front': 0.0,
            'left_wheel_back': 0.0,
            'right_wheel_back': 0.0
        }
        
        self.wheel_velocities = {
            'left_wheel_front': 0.0,
            'right_wheel_front': 0.0,
            'left_wheel_back': 0.0,
            'right_wheel_back': 0.0
        }
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            qos_profile
        )
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer
        self.create_timer(0.01, self.timer_callback)  # 100Hz update rate
        
        self.get_logger().info('Wheel encoder node initialized')

    def read_encoder_data(self):
        try:
            # Wait for header
            while True:
                if self.serial_port.read(1) == self.HEADER[0:1]:
                    if self.serial_port.read(1) == self.HEADER[1:2]:
                        break
            
            # Read encoder data - modify this according to your serial protocol
            # Assuming the protocol sends 8 bytes (2 bytes per wheel encoder)
            data = self.serial_port.read(8)
            if len(data) != 8:
                return False
            
            # Parse encoder ticks (modify according to your protocol)
            fl_ticks = int.from_bytes(data[0:2], byteorder='little', signed=True)
            fr_ticks = int.from_bytes(data[2:4], byteorder='little', signed=True)
            rl_ticks = int.from_bytes(data[4:6], byteorder='little', signed=True)
            rr_ticks = int.from_bytes(data[6:8], byteorder='little', signed=True)
            
            # Convert ticks to radians
            ticks_to_rad = 2 * math.pi / self.ENCODER_TICKS_PER_REVOLUTION
            
            # Update wheel positions
            self.wheel_positions['left_wheel_front'] = fl_ticks * ticks_to_rad
            self.wheel_positions['right_wheel_front'] = fr_ticks * ticks_to_rad
            self.wheel_positions['left_wheel_back'] = rl_ticks * ticks_to_rad
            self.wheel_positions['right_wheel_back'] = rr_ticks * ticks_to_rad
            
            self.get_logger().info(f"Wheel positions: {self.wheel_positions}")
            return True
            
        except Exception as e:
            self.get_logger().error(f'Serial read error: {str(e)}')
            return False

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Add joint names
        msg.name = list(self.wheel_positions.keys())
        
        # Add positions
        msg.position = list(self.wheel_positions.values())
        
        # Add velocities
        msg.velocity = list(self.wheel_velocities.values())
        
        # Publish
        self.joint_state_pub.publish(msg)

    def publish_wheel_transforms(self):
        now = self.get_clock().now().to_msg()
        
        # Publish transform for each wheel
        for wheel_name, position in self.wheel_positions.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = f"{wheel_name}_joint"
            t.child_frame_id = wheel_name
            
            # Set the rotation of the wheel
            t.transform.rotation.x = math.sin(position / 2.0)
            t.transform.rotation.w = math.cos(position / 2.0)
            
            self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        if self.read_encoder_data():
            self.publish_joint_states()
            self.publish_wheel_transforms()

def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
