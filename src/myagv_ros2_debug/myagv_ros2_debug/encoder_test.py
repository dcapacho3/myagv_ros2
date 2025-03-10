#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
import serial
import math
import numpy as np
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MyAGVNode(Node):
    def __init__(self):
        super().__init__('myagv_node')

        # Constants
        self.HEADER = bytes([0xfe, 0xfe])
        self.TOTAL_RECEIVE_SIZE = 18
        self.sample_freq = 50.0  # 200 Hz sampling rate

        # Movement parameters
        self.linear_deadzone = 0.01  # 1cm/s
        self.angular_deadzone = 0.01  # ~0.6 degrees/s
        self.cmd_smoothing_factor = 0.7
        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 2.0  # rad/s
        self.max_linear_accel = 2.0  # m/s²
        self.max_angular_accel = 4.0  # rad/s²

        # Command smoothing variables
        self.last_cmd_x = 0.0
        self.last_cmd_y = 0.0
        self.last_cmd_theta = 0.0
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vtheta = 0.0

        # Initialize serial port
        self.serial_port = serial.Serial(
            port='/dev/ttyAMA2',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.1
        )
        
        # Clear buffers
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        # Initialize variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.last_time = self.get_clock().now()
        self.last_gyro_update = None
        self.battery_voltage = 0.0
        
        # Initialize IMU data storage
        self.ax = 0.0  # Accelerometer readings
        self.ay = 0.0
        self.az = 0.0
        self.gx = 0.0  # Gyroscope readings
        self.gy = 0.0
        self.gz = 0.0
        
        # Initialize dual orientation system with more conservative parameters
        self.wheel_based_theta = 0.0
        self.imu_theta = 0.0
        self.vel_window_size = 10  # Increased window size for better stability
        self.vx_window = []
        self.vy_window = []
        self.min_velocity_threshold = 0.05  # Increased threshold to avoid noise
        self.last_valid_wheel_theta = 0.0  # Store last valid wheel-based orientation
        self.wheel_theta_timeout = 0.5  # Time window to maintain last valid orientation
        self.last_wheel_update = self.get_clock().now()
        
        # Initialize orientation filter
        self.alpha = 0.1  # Filter coefficient for orientation
        self.filtered_gyro = 0.0

        # Initialize gyro bias calibration
        self.gyro_bias = 0.0
        self.calibration_samples = 200
        self.calibrated = False
        self.calibration_sum = 0.0

        # Covariance matrices
        self.odom_pose_covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.001
        ]
        self.odom_twist_covariance = self.odom_pose_covariance.copy()

        # QoS profile for reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pub_imu = self.create_publisher(Imu, 'imu_data', qos_profile)
        self.pub_odom = self.create_publisher(Odometry, 'odom', qos_profile)
        self.pub_voltage = self.create_publisher(Float32, 'Voltage', 10)
        self.pub_status = self.create_publisher(String, 'robot_status', 10)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel_out',
            self.cmd_vel_callback,
            10
        )

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for main loop
        self.create_timer(1.0/self.sample_freq, self.timer_callback)

        # Command variables
        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_angular_z = 0.0

        self.get_logger().info('MyAGV node initialized - Starting calibration...')
        self.publish_status("Calibrating gyroscope - Please keep the robot still...")

    def publish_status(self, status_msg):
        """Publish robot status message"""
        msg = String()
        msg.data = status_msg
        self.pub_status.publish(msg)
        self.get_logger().info(status_msg)

    def apply_deadzone(self, value, deadzone):
        """Apply deadzone to value"""
        return 0.0 if abs(value) < deadzone else value

    def limit_acceleration(self, current, target, max_accel, dt):
        """Limit acceleration"""
        max_change = max_accel * dt
        change = target - current
        return current + np.clip(change, -max_change, max_change)

    def smooth_commands(self, target_x, target_y, target_theta):
        """Apply exponential smoothing to commands"""
        self.last_cmd_x = self.cmd_smoothing_factor * target_x + \
                         (1 - self.cmd_smoothing_factor) * self.last_cmd_x
        self.last_cmd_y = self.cmd_smoothing_factor * target_y + \
                         (1 - self.cmd_smoothing_factor) * self.last_cmd_y
        self.last_cmd_theta = self.cmd_smoothing_factor * target_theta + \
                             (1 - self.cmd_smoothing_factor) * self.last_cmd_theta
        return self.last_cmd_x, self.last_cmd_y, self.last_cmd_theta

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        if not self.calibrated:
            self.get_logger().warning('Cannot move yet - Gyroscope still calibrating')
            return
            
        self.cmd_linear_x = msg.linear.x
        self.cmd_linear_y = msg.linear.y
        self.cmd_angular_z = msg.angular.z

    def read_serial_data(self):
        """Read and parse serial data"""
        try:
            # Wait for header
            while True:
                if self.serial_port.in_waiting < 2:
                    return False
                if self.serial_port.read(1) == self.HEADER[0:1]:
                    if self.serial_port.read(1) == self.HEADER[1:2]:
                        break

            if self.serial_port.in_waiting < self.TOTAL_RECEIVE_SIZE:
                return False

            data = self.serial_port.read(self.TOTAL_RECEIVE_SIZE)
            if len(data) != self.TOTAL_RECEIVE_SIZE:
                return False

            # Parse velocities with thresholds
            raw_vx = (data[0] - 128)
            raw_vy = (data[1] - 128)
            raw_vtheta = (data[2] - 128)
            
            # Apply thresholds and scaling
            self.vx = 0.01 * raw_vx if abs(raw_vx) > 2 else 0.0
            self.vy = 0.01 * raw_vy if abs(raw_vy) > 2 else 0.0
            self.vtheta = 0.01 * raw_vtheta if abs(raw_vtheta) > 2 else 0.0

            # Parse all IMU data
            # Accelerometer (m/s²)
            raw_ax = ((data[3] + data[4] * 256) - 10000)
            raw_ay = ((data[5] + data[6] * 256) - 10000)
            raw_az = ((data[7] + data[8] * 256) - 10000)
            
            self.ax = raw_ax * 0.001  # Convert to m/s²
            self.ay = raw_ay * 0.001
            self.az = raw_az * 0.001
            
            # Gyroscope (rad/s)
            raw_gx = ((data[9] + data[10] * 256) - 10000)
            raw_gy = ((data[11] + data[12] * 256) - 10000)
            raw_gz = ((data[13] + data[14] * 256) - 10000)
            
            # Convert to radians/second
            self.gx = raw_gx * 0.1 * math.pi / 180.0
            self.gy = raw_gy * 0.1 * math.pi / 180.0
            self.gz = raw_gz * 0.1 * math.pi / 180.0

            # Keep existing gyro calculation for now
            gyro_z = raw_gz * 0.1 * math.pi / 180.0  # Convert to rad/s

            # Calibrate gyro if needed
            if not self.calibrated:
                self.calibration_sum += gyro_z
                self.calibration_samples -= 1
                if self.calibration_samples == 0:
                    self.gyro_bias = self.calibration_sum / 200
                    self.calibrated = True
                    self.filtered_gyro = 0.0  # Initialize filtered value
                    self.publish_status("Calibration complete - Robot ready to move!")
                return True

            # Apply gyro bias compensation and filtering
            gyro_z -= self.gyro_bias
            
            # Apply low-pass filter to gyro data
            self.filtered_gyro = (1 - self.alpha) * self.filtered_gyro + self.alpha * gyro_z

            # Update theta using filtered gyro
            current_time = self.get_clock().now()
            if self.last_gyro_update is not None:
                dt = (current_time - self.last_gyro_update).nanoseconds / 1e9
                # Use filtered gyro for orientation update
                delta_theta = self.filtered_gyro * dt
                self.theta += delta_theta
                # Normalize theta to [-π, π]
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            self.last_gyro_update = current_time

            # Battery voltage
            self.battery_voltage = data[16] / 10.0

            return True

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {str(e)}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {str(e)}')
            return False

    def update_odometry(self, dt):
        """Update odometry based on velocities"""
        if not self.calibrated:
            return

        # Apply deadzone to velocities
        vx = self.apply_deadzone(self.vx, self.linear_deadzone)
        vy = self.apply_deadzone(self.vy, self.linear_deadzone)
        vtheta = self.apply_deadzone(self.vtheta, self.angular_deadzone)

        # Update position using velocities and current orientation
        if abs(vx) > 0.0 or abs(vy) > 0.0:
            # Transform velocities from robot frame to world frame
            delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
            delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
            
            self.x += delta_x
            self.y += delta_y

        # Debug output
        if self.get_logger().get_effective_level() <= 20:  # DEBUG level
            self.get_logger().debug(
                f'Odom - Yaw: {math.degrees(self.theta):.2f}°, ' +
                f'X: {self.x:.3f}, Y: {self.y:.3f}, ' +
                f'VX: {vx:.3f}, VY: {vy:.3f}, ' +
                f'VTheta: {vtheta:.3f}, ' +
                f'Filtered Gyro: {math.degrees(self.filtered_gyro):.2f}°/s'
            )

    def publish_tf(self):
        """Publish transform from odom to base_footprint"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Create quaternion from yaw
        w = math.cos(self.theta / 2.0)
        z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = w
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = z

        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set orientation
        w = math.cos(self.theta / 2.0)
        z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = w
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = z

        # Set velocities in robot's frame
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta

        # Set covariances
        odom.pose.covariance = self.odom_pose_covariance
        odom.twist.covariance = self.odom_twist_covariance

        self.pub_odom.publish(odom)

    def write_speed(self, vx, vy, vtheta):
        """Send speed commands to the robot"""
        if not self.calibrated:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Apply deadzone
        vx = self.apply_deadzone(vx, self.linear_deadzone)
        vy = self.apply_deadzone(vy, self.linear_deadzone)
        vtheta = self.apply_deadzone(vtheta, self.angular_deadzone)

        # Apply command smoothing
        vx, vy, vtheta = self.smooth_commands(vx, vy, vtheta)

        # Limit acceleration
        vx = self.apply_deadzone(vx, self.linear_deadzone)
        vy = self.apply_deadzone(vy, self.linear_deadzone)
        vtheta = self.apply_deadzone(vtheta, self.angular_deadzone)

        # Apply command smoothing
        vx, vy, vtheta = self.smooth_commands(vx, vy, vtheta)

        # Limit acceleration
        vx = self.limit_acceleration(self.prev_vx, vx, self.max_linear_accel, dt)
        vy = self.limit_acceleration(self.prev_vy, vy, self.max_linear_accel, dt)
        vtheta = self.limit_acceleration(self.prev_vtheta, vtheta, self.max_angular_accel, dt)

        # Update previous velocities
        self.prev_vx = vx
        self.prev_vy = vy
        self.prev_vtheta = vtheta

        # Clamp final values
        vx = np.clip(vx, -self.max_linear_velocity, self.max_linear_velocity)
        vy = np.clip(vy, -self.max_linear_velocity, self.max_linear_velocity)
        vtheta = np.clip(vtheta, -self.max_angular_velocity, self.max_angular_velocity)

        try:
            # Convert to bytes with improved scaling
            x_send = int(vx * 100) + 128
            y_send = int(vy * 100) + 128
            rot_send = int(vtheta * 100) + 128
            
            # Ensure values are within valid range
            x_send = np.clip(x_send, 0, 255)
            y_send = np.clip(y_send, 0, 255)
            rot_send = np.clip(rot_send, 0, 255)
            
            checksum = (x_send + y_send + rot_send) & 0xFF

            # Send command twice for reliability
            msg = bytes([0xfe, 0xfe, x_send, y_send, rot_send, checksum])
            self.serial_port.write(msg)
            self.serial_port.write(msg)
            self.serial_port.flush()
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error writing speed: {str(e)}')

    def publish_voltage(self):
        """Publish battery voltage"""
        msg = Float32()
        msg.data = float(self.battery_voltage)
        self.pub_voltage.publish(msg)

    def timer_callback(self):
        """Main control loop"""
        try:
            if self.read_serial_data():
                current_time = self.get_clock().now()
                dt = (current_time - self.last_time).nanoseconds / 1e9

                # Update odometry and publish data
                if self.calibrated:
                    self.update_odometry(dt)
                    self.publish_tf()
                    self.publish_odom()
                    self.publish_voltage()

                    # Send commands to the robot
                    self.write_speed(self.cmd_linear_x, self.cmd_linear_y, self.cmd_angular_z)

                self.last_time = current_time

        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MyAGVNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
