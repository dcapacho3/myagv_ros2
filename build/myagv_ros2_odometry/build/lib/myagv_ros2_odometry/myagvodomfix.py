#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import serial
import math
import numpy as np
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class IMUIntegrator:
    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion [w, x, y, z]
        self.beta = 0.033  # Reduced for better stability
        self.sample_period = 1.0/200.0  # 200 Hz sample rate
        self.gyro_bias = np.zeros(3)
        self.calibration_samples = 100
        self.calibrated = False

    def calibrate_gyro(self, gx, gy, gz):
        """Calibrate gyroscope bias"""
        if not self.calibrated:
            self.gyro_bias += np.array([gx, gy, gz])
            self.calibration_samples -= 1
            if self.calibration_samples == 0:
                self.gyro_bias /= 100
                self.calibrated = True
                return True
        return False

    def update(self, gx, gy, gz, ax, ay, az):
        """Update orientation using IMU data with Madgwick filter"""
        # Calibrate if needed
        if not self.calibrated:
            if not self.calibrate_gyro(gx, gy, gz):
                return

        # Apply bias compensation
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]

        q = self.q

        # Normalize accelerometer measurements
        norm = np.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0:
            return
        ax /= norm
        ay /= norm
        az /= norm

        # Convert gyroscope readings to rad/s
        gx = np.radians(gx)
        gy = np.radians(gy)
        gz = np.radians(gz)

        # Gradient descent algorithm
        F = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - ax,
            2*(q[0]*q[1] + q[2]*q[3]) - ay,
            2*(0.5 - q[1]**2 - q[2]**2) - az
        ])
        
        J = np.array([
            [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
            [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
            [0, -4*q[1], -4*q[2], 0]
        ])

        step = J.T @ F
        step = step / np.linalg.norm(step)

        # Rate of change of quaternion
        qdot = 0.5 * np.array([
            -q[1]*gx - q[2]*gy - q[3]*gz,
            q[0]*gx + q[2]*gz - q[3]*gy,
            q[0]*gy - q[1]*gz + q[3]*gx,
            q[0]*gz + q[1]*gy - q[2]*gx
        ]) - self.beta * step

        # Integrate to get new quaternion
        self.q = q + qdot * self.sample_period
        self.q = self.q / np.linalg.norm(self.q)

    def get_euler(self):
        """Get Euler angles (roll, pitch, yaw) from quaternion"""
        r = Rotation.from_quat([self.q[1], self.q[2], self.q[3], self.q[0]])
        return r.as_euler('xyz')

class MyAGVNode(Node):
    def __init__(self):
        super().__init__('myagv_node')

        # Constants
        self.HEADER = bytes([0xfe, 0xfe])
        self.TOTAL_RECEIVE_SIZE = 18
        self.sample_freq = 200.0  # Increased to 200Hz

        # Movement parameters
        self.linear_deadzone = 0.02  # 2cm/s
        self.angular_deadzone = 0.02  # 0.02 rad/s
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

        # Initialize serial port with larger buffer
        self.serial_port = serial.Serial(
            port='/dev/ttyAMA2',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.1,
            inter_byte_timeout=None
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
        self.imu_integrator = IMUIntegrator()
        self.battery_voltage = 0.0

        # Covariance matrices
        self.odom_pose_covariance = np.array([
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e-3, 0, 0, 0,
            0, 0, 0, 1e-3, 0, 0,
            0, 0, 0, 0, 1e-3, 0,
            0, 0, 0, 0, 0, 1e-3
        ], dtype=float)
        self.odom_twist_covariance = self.odom_pose_covariance.copy()

        # QoS profile for better reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pub_imu = self.create_publisher(Imu, 'imu_data', qos_profile)
        self.pub_odom = self.create_publisher(Odometry, 'odom', qos_profile)
        self.pub_voltage = self.create_publisher(Float32, 'Voltage', 10)

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

        self.get_logger().info('MyAGV node initialized')

    def apply_deadzone(self, value, deadzone):
        """Apply deadzone to remove noise in small movements"""
        return 0.0 if abs(value) < deadzone else value

    def limit_acceleration(self, current, target, max_accel, dt):
        """Limit acceleration to prevent sudden movements"""
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
        self.cmd_linear_x = msg.linear.x
        self.cmd_linear_y = msg.linear.y
        self.cmd_angular_z = msg.angular.z

    def read_serial_data(self):
        """Read and parse serial data from the robot"""
        try:
            # Wait for header
            while True:
                if self.serial_port.in_waiting < 2:
                    return False
                if self.serial_port.read(1) == self.HEADER[0:1]:
                    if self.serial_port.read(1) == self.HEADER[1:2]:
                        break

            # Check if enough data is available
            if self.serial_port.in_waiting < self.TOTAL_RECEIVE_SIZE:
                return False

            # Read data
            data = self.serial_port.read(self.TOTAL_RECEIVE_SIZE)
            if len(data) != self.TOTAL_RECEIVE_SIZE:
                return False

            # Parse velocities
            self.vx = (data[0] - 128) * 0.01
            self.vy = (data[1] - 128) * 0.01
            self.vtheta = (data[2] - 128) * 0.01

            # Parse IMU data
            acc_x = ((data[3] + data[4] * 256) - 10000) * 0.001
            acc_y = ((data[5] + data[6] * 256) - 10000) * 0.001
            acc_z = ((data[7] + data[8] * 256) - 10000) * 0.001

            gyro_x = ((data[9] + data[10] * 256) - 10000) * 0.1
            gyro_y = ((data[11] + data[12] * 256) - 10000) * 0.1
            gyro_z = ((data[13] + data[14] * 256) - 10000) * 0.1

            # Battery voltage
            self.battery_voltage = data[16] / 10.0

            # Update IMU orientation
            self.imu_msg = Imu()
            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.imu_msg.header.frame_id = 'imu'

            self.imu_msg.linear_acceleration.x = acc_x
            self.imu_msg.linear_acceleration.y = acc_y
            self.imu_msg.linear_acceleration.z = acc_z
            self.imu_msg.angular_velocity.x = gyro_x
            self.imu_msg.angular_velocity.y = gyro_y
            self.imu_msg.angular_velocity.z = gyro_z

            # Update IMU orientation
            self.imu_integrator.update(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z)
            _, _, yaw = self.imu_integrator.get_euler()
            
            # Convert to quaternion
            q = Rotation.from_euler('z', yaw).as_quat()
            self.imu_msg.orientation.x = q[0]
            self.imu_msg.orientation.y = q[1]
            self.imu_msg.orientation.z = q[2]
            self.imu_msg.orientation.w = q[3]

            return True

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {str(e)}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {str(e)}')
            return False

    def write_speed(self, vx, vy, vtheta):
        """Send speed commands to the robot"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Apply deadzone
        vx = self.apply_deadzone(vx, self.linear_deadzone)
        vy = self.apply_deadzone(vy, self.linear_deadzone)
        vtheta = self.apply_deadzone(vtheta, self.angular_deadzone)

        # Smooth commands
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
            # Convert to bytes
            x_send = int(vx * 100) + 128
            y_send = int(vy * 100) + 128
            rot_send = int(vtheta * 100) + 128
            checksum = (x_send + y_send + rot_send) & 0xFF

            # Construct and send message
            msg = bytes([0xfe, 0xfe, x_send, y_send, rot_send, checksum])
            self.serial_port.write(msg)
            self.serial_port.write(msg)
            self.serial_port.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error writing speed: {str(e)}')

    def update_odometry(self, dt):
        """Update odometry based on wheel velocities and IMU orientation"""
        # Get current orientation from IMU integrator
        _, _, yaw = self.imu_integrator.get_euler()
        self.theta = yaw

        # Update position based on wheel odometry and IMU orientation
        # Using IMU for orientation makes the position estimate more accurate
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt

        self.x += delta_x
        self.y += delta_y

    def publish_tf(self):
        """Publish transform from odom to base_footprint"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = Rotation.from_euler('z', self.theta).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

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
        q = Rotation.from_euler('z', self.theta).as_quat()
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set velocities
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta

        # Set covariances
        odom.pose.covariance = self.odom_pose_covariance
        odom.twist.covariance = self.odom_twist_covariance

        self.pub_odom.publish(odom)

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

                # Update odometry
                self.update_odometry(dt)

                # Publish all data
                self.publish_tf()
                self.publish_odom()
                self.pub_imu.publish(self.imu_msg)
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
