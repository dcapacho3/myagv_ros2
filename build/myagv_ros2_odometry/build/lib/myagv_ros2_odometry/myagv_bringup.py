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
        self.sample_freq = 200.0  # 200 Hz sampling rate

        # Enhanced movement parameters for drift correction
        self.linear_deadzone = 0.015  # Increased to reduce noise
        self.angular_deadzone = 0.015  # Increased for better stability
        self.cmd_smoothing_factor = 0.85  # Increased for smoother motion
        self.max_linear_velocity = 0.8  # Reduced for better control
        self.max_angular_velocity = 1.5  # Reduced for stability
        self.max_linear_accel = 1.0  # More conservative
        self.max_angular_accel = 2.0  # More conservative
        
        # IMU drift correction parameters
        self.gyro_drift_threshold = 0.002  # rad/s
        self.stationary_threshold = 0.005  # m/s
        self.imu_weight = 0.85  # Weight for IMU in sensor fusion
        self.motion_confidence = 1.0  # Dynamic confidence tracking

        # Odometry and velocity filtering parameters
        self.velocity_filter_alpha = 0.3  # Adjust based on noise level
        self.max_acceleration = 2.0  # m/s², adjust based on robot capabilities
        self.velocity_jump_threshold = 0.5  # m/s
        self.position_update_threshold = 0.1  # Maximum allowed position change per update (m)

        # Command smoothing variables
        self.last_cmd_x = 0.0
        self.last_cmd_y = 0.0
        self.last_cmd_theta = 0.0
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vtheta = 0.0

        # Initialize main serial port for motor control
        self.serial_port = serial.Serial(
            port='/dev/ttyAMA2',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=0.1
        )
        
        # Initialize Arduino serial port for IMU and weight data
        self.arduino_port = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=230400,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
        
        # Clear buffers for both ports
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        self.arduino_port.reset_input_buffer()
        self.arduino_port.reset_output_buffer()

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
        
        # Initialize orientation filter
        self.alpha = 0.1  # Filter coefficient for orientation
        self.filtered_gyro = 0.0

        # Initialize velocity filters
        self.velocity_filter_alpha = 0.3  # Filter coefficient for velocities
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_vtheta = 0.0
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vtheta = 0.0
        
        # Initialize acceleration values
        self.ax_world = 0.0
        self.ay_world = 0.0
        self.last_ax_world = 0.0
        self.last_ay_world = 0.0
        self.integrated_vx = 0.0
        self.integrated_vy = 0.0

        # Initialize gyro bias calibration
        self.gyro_bias = 0.0
        self.calibration_samples = 200
        self.calibrated = False
        self.calibration_sum = 0.0
        self.last_progress_update = self.get_clock().now()

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
        self.pub_weight1 = self.create_publisher(Float32, 'weight1', 10)
        self.pub_weight2 = self.create_publisher(Float32, 'weight2', 10)
        
        # IMU scaling factors
        self.ACCEL_SCALE = 16384.0  # For ±2g range
        self.GYRO_SCALE = 131.0     # For ±250°/s range

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

    def validate_velocity(self, current_vel, last_vel, dt):
        """
        Validate velocity readings using multiple criteria
        """
        if dt <= 0:
            return last_vel

        # Calculate acceleration
        accel = (current_vel - last_vel) / dt

        # Check for physically impossible accelerations
        if abs(accel) > self.max_acceleration:
            self.get_logger().warning(f'Excessive acceleration detected: {accel:.2f} m/s²')
            # Limit the velocity change based on max acceleration
            max_vel_change = self.max_acceleration * dt
            return last_vel + np.clip(current_vel - last_vel, -max_vel_change, max_vel_change)

        # Check for sudden velocity jumps
        if abs(current_vel - last_vel) > self.velocity_jump_threshold:
            self.get_logger().warning(f'Velocity jump detected: {abs(current_vel - last_vel):.2f} m/s')
            return last_vel

        return current_vel

    def filter_velocity(self, current_vel, filtered_vel, dt):
        """
        Apply adaptive filtering to velocity measurements with enhanced straight-line detection
        """
        # Base filter coefficient
        base_alpha = self.velocity_filter_alpha
        
        # Detect if we're in a straight-line movement
        vel_magnitude = abs(current_vel)
        vel_change = abs(current_vel - filtered_vel)
        
        # Very small velocity - likely noise or trying to stop
        if vel_magnitude < 0.02:
            alpha = base_alpha * 0.3  # Very strong filtering
            if vel_magnitude < 0.01:
                return 0.0  # Force zero for very small velocities
        # Straight line detection - steady velocity
        elif vel_change < 0.03:
            alpha = base_alpha * 1.2  # Reduce filtering to maintain straight line
        # Normal movement
        else:
            alpha = base_alpha
            
        # Additional filtering during direction changes
        if (current_vel * filtered_vel) < 0:  # Direction change
            alpha *= 0.4  # Strong filtering during direction changes
            
        # Apply smoothing
        filtered = alpha * current_vel + (1 - alpha) * filtered_vel
        
        # If almost stopped, force to zero
        if abs(filtered) < 0.005:
            filtered = 0.0
            
        return filtered

    def transform_acceleration(self, ax, ay, theta):
        """Transform acceleration from robot frame to world frame"""
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        ax_world = ax * cos_theta - ay * sin_theta
        ay_world = ax * sin_theta + ay * cos_theta
        return ax_world, ay_world

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

    def detect_stationary_state(self):
        """Detect if robot is stationary to help with drift correction"""
        vel_magnitude = math.sqrt(self.vx**2 + self.vy**2)
        return vel_magnitude < self.stationary_threshold

    def correct_gyro_drift(self, gz):
        """Enhanced gyro drift correction"""
        if self.detect_stationary_state():
            if abs(gz) < self.gyro_drift_threshold:
                return 0.0
            else:
                # Apply stronger filtering when stationary but detecting motion
                gz *= 0.3
        return gz

    def read_serial_data(self):
        """Read and parse serial data from both Arduino and motor controller with enhanced drift correction"""
        try:
            # Read Arduino IMU and weight data
            if self.arduino_port.in_waiting > 0:
                arduino_data = self.arduino_port.readline().decode().strip()
                
                # Skip debug messages
                if arduino_data and not arduino_data.startswith("[DEBUG]"):
                    try:
                        values = [float(x) for x in arduino_data.split(',')]
                        if len(values) == 8:
                            # Convert accelerometer data from raw to g's and then to m/s²
                            ax = (values[0] / self.ACCEL_SCALE) * 9.81
                            ay = (values[1] / self.ACCEL_SCALE) * 9.81
                            az = (values[2] / self.ACCEL_SCALE) * 9.81
                            
                            # Convert gyroscope data from raw to rad/s
                            gx = values[3] * math.pi / (180.0 * self.GYRO_SCALE)
                            gy = values[4] * math.pi / (180.0 * self.GYRO_SCALE)
                            gz = values[5] * math.pi / (180.0 * self.GYRO_SCALE)
                            
                            # Extract weight data
                            weight1 = values[6]
                            weight2 = values[7]
                            
                            # Transform acceleration to world frame and store
                            self.ax_world, self.ay_world = self.transform_acceleration(ax, ay, self.theta)
                            
                            # Apply complementary filter to acceleration
                            accel_filter_alpha = 0.1
                            self.ax_world = accel_filter_alpha * self.ax_world + \
                                          (1 - accel_filter_alpha) * self.last_ax_world
                            self.ay_world = accel_filter_alpha * self.ay_world + \
                                          (1 - accel_filter_alpha) * self.last_ay_world
                            
                            # Store current accelerations for next iteration
                            self.last_ax_world = self.ax_world
                            self.last_ay_world = self.ay_world

                            self.publish_weights(weight1, weight2)

                            # Update orientation using filtered gyro data
                            if not self.calibrated:
                                self.calibration_sum += gz
                                self.calibration_samples -= 1
                                
                                progress = ((200 - self.calibration_samples) / 200) * 100
                                if self.calibration_samples % 20 == 0:
                                    self.publish_status(f"Calibrating gyroscope - {progress:.0f}% complete...")
                                
                                if self.calibration_samples == 0:
                                    self.gyro_bias = self.calibration_sum / 200
                                    self.calibrated = True
                                    self.filtered_gyro = 0.0
                                    self.publish_status("Calibration complete - Robot ready to move!")
                                    self.get_logger().info(f'Gyro calibration complete. Bias: {self.gyro_bias}')
                            else:
                                # Apply gyro bias compensation and filtering
                                gz -= self.gyro_bias
                                self.filtered_gyro = (1 - self.alpha) * self.filtered_gyro + self.alpha * gz

                                # Update theta using filtered gyro
                                current_time = self.get_clock().now()
                                if self.last_gyro_update is not None:
                                    dt = (current_time - self.last_gyro_update).nanoseconds / 1e9
                                    delta_theta = self.filtered_gyro * dt
                                    self.theta += delta_theta
                                    self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
                                    
                                self.last_gyro_update = current_time
                                self.publish_imu_data(ax, ay, az, gx, gy, gz)

                    except ValueError as e:
                        self.get_logger().warning(f'Error parsing Arduino data: {arduino_data} - {str(e)}')
                        return False

            # Read motor controller data - Verify proper message format
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

            # Parse velocities with thresholds and debugging
            raw_vx = (data[0] - 128)
            raw_vy = (data[1] - 128)
            raw_vtheta = (data[2] - 128)
            
            # Apply thresholds and scaling with focused debugging
            self.vx = 0.01 * raw_vx if abs(raw_vx) > 2 else 0.0
            self.vy = 0.01 * raw_vy if abs(raw_vy) > 2 else 0.0
            self.vtheta = 0.01 * raw_vtheta if abs(raw_vtheta) > 2 else 0.0
            
            # Only log if there's significant movement
            if abs(self.vx) > 0.01 or abs(self.vy) > 0.01 or abs(self.vtheta) > 0.01:
                self.get_logger().info(f'VEL: vx={self.vx:.3f} vy={self.vy:.3f} vth={self.vtheta:.3f}')

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
        """Update odometry using enhanced sensor fusion and drift correction"""
        if not self.calibrated or dt <= 0:
            return

        # Enhanced stationary detection and drift correction
        is_stationary = self.detect_stationary_state()
        if is_stationary:
            self.vx = 0.0
            self.vy = 0.0
            self.vtheta = 0.0
            return

        # Apply deadzone with confidence tracking
        vx = self.apply_deadzone(self.vx, self.linear_deadzone)
        vy = self.apply_deadzone(self.vy, self.linear_deadzone)
        vtheta = self.apply_deadzone(self.vtheta, self.angular_deadzone)

        # Enhanced velocity validation
        current_time = self.get_clock().now()
        dt_velocity = (current_time - self.last_time).nanoseconds / 1e9
        
        # Validate velocities with improved filtering
        vx = self.validate_velocity(vx, self.last_vx, dt_velocity)
        vy = self.validate_velocity(vy, self.last_vy, dt_velocity)
        vtheta = self.validate_velocity(vtheta, self.last_vtheta, dt_velocity)

        # Apply enhanced motion confidence
        confidence_scale = min(1.0, self.motion_confidence)
        vx *= confidence_scale
        vy *= confidence_scale
        vtheta *= confidence_scale
        if not self.calibrated or dt <= 0:
            return

        # Apply deadzone to velocities
        vx = self.apply_deadzone(self.vx, self.linear_deadzone)
        vy = self.apply_deadzone(self.vy, self.linear_deadzone)
        vtheta = self.apply_deadzone(self.vtheta, self.angular_deadzone)

        # Store raw velocities for debugging
        raw_vx, raw_vy, raw_vtheta = vx, vy, vtheta

        # Validate and filter velocities
        current_time = self.get_clock().now()
        dt_velocity = (current_time - self.last_time).nanoseconds / 1e9
        
        # Validate velocities using acceleration limits and jump detection
        vx = self.validate_velocity(vx, self.last_vx, dt_velocity)
        vy = self.validate_velocity(vy, self.last_vy, dt_velocity)
        vtheta = self.validate_velocity(vtheta, self.last_vtheta, dt_velocity)
        
        # Apply exponential smoothing to velocities with time step
        self.filtered_vx = self.filter_velocity(vx, self.filtered_vx, dt_velocity)
        self.filtered_vy = self.filter_velocity(vy, self.filtered_vy, dt_velocity)
        self.filtered_vtheta = self.filter_velocity(vtheta, self.filtered_vtheta, dt_velocity)

        # Only log significant changes in filtered velocities
        if (abs(self.filtered_vx - self.last_vx) > 0.05 or 
            abs(self.filtered_vy - self.last_vy) > 0.05 or 
            abs(self.filtered_vtheta - self.last_vtheta) > 0.05):
            self.get_logger().info(
                f'FILT: vx={self.filtered_vx:.3f} vy={self.filtered_vy:.3f} vth={self.filtered_vtheta:.3f}'
            )
        
        # Store current velocities for next iteration
        self.last_vx = vx
        self.last_vy = vy
        self.last_vtheta = vtheta

        # Transform velocities from robot frame to world frame
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        
        # Calculate position change using filtered velocities
        delta_x = (self.filtered_vx * cos_theta - self.filtered_vy * sin_theta) * dt
        delta_y = (self.filtered_vx * sin_theta + self.filtered_vy * cos_theta) * dt

        # Add additional sanity check for position updates
        max_position_change = self.max_linear_velocity * dt * 1.2  # 20% margin
        if abs(delta_x) > max_position_change or abs(delta_y) > max_position_change:
            self.get_logger().warning(
                f'Large position change detected - dx: {delta_x:.4f}, dy: {delta_y:.4f}'
            )
            # Limit the position change
            scale_factor = max_position_change / max(abs(delta_x), abs(delta_y))
            delta_x *= scale_factor
            delta_y *= scale_factor

        # Update position
        self.x += delta_x
        self.y += delta_y

        # Log position updates only when there's significant movement
        if abs(delta_x) > 0.01 or abs(delta_y) > 0.01:
            self.get_logger().info(
                f'POS: x={self.x:.3f} y={self.y:.3f} θ={math.degrees(self.theta):.1f}° ' +
                f'Δ(x={delta_x:.3f} y={delta_y:.3f})'
            )

    def publish_status(self, status_msg):
        """Publish robot status message"""
        msg = String()
        msg.data = status_msg
        self.pub_status.publish(msg)
        self.get_logger().info(status_msg)

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
        odom.twist.twist.linear.x = self.filtered_vx  # Use filtered velocities
        odom.twist.twist.linear.y = self.filtered_vy
        odom.twist.twist.angular.z = self.filtered_vtheta

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
            
            self.get_logger().debug(
                f'Sent command - vx: {vx:.4f}, vy: {vy:.4f}, vtheta: {vtheta:.4f}\n'
                f'Bytes sent: {[x_send, y_send, rot_send]}'
            )
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error writing speed: {str(e)}')

    def publish_imu_data(self, ax, ay, az, gx, gy, gz):
        """Publish IMU data"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Set linear acceleration
        msg.linear_acceleration.x = ax * 9.81  # Convert to m/s²
        msg.linear_acceleration.y = ay * 9.81
        msg.linear_acceleration.z = az * 9.81

        # Set angular velocity
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Set 9-element covariance matrices (3x3)
        imu_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        msg.linear_acceleration_covariance = imu_covariance
        msg.angular_velocity_covariance = imu_covariance
        msg.orientation_covariance = [-1.0] * 9  # Orientation not provided by this IMU

        self.pub_imu.publish(msg)

    def publish_weights(self, weight1, weight2):
        """Publish weight sensor data"""
        try:
            msg1 = Float32()
            msg1.data = float(weight1)
            self.pub_weight1.publish(msg1)

            msg2 = Float32()
            msg2.data = float(weight2)
            self.pub_weight2.publish(msg2)
            
            self.get_logger().debug(f'Published weights - Load1: {weight1:.3f}, Load2: {weight2:.2f}')
        except Exception as e:
            self.get_logger().error(f'Error publishing weights: {str(e)}')

    def publish_voltage(self):
        """Publish battery voltage"""
        msg = Float32()
        msg.data = float(self.battery_voltage)
        self.pub_voltage.publish(msg)

    def timer_callback(self):
        """Main control loop"""
        try:
            # Check calibration status and update progress regularly
            if not self.calibrated:
                current_time = self.get_clock().now()
                if (current_time - self.last_progress_update).nanoseconds >= 1e9:  # Update every second
                    progress = ((200 - self.calibration_samples) / 200) * 100
                    if progress < 100:  # Only show progress if not complete
                        self.publish_status(f"Calibrating gyroscope - {progress:.0f}% complete... Please keep robot still")
                    self.last_progress_update = current_time

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
