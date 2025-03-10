// myagv.cpp
#include "myagv_ros2_odom/myagv_odom.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

using namespace std::chrono_literals;

MyAGVNode::MyAGVNode() 
: Node("myagv_node"),
  last_time_(this->now())
{
    // Setup QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();

    // Initialize publishers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_data", qos);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("voltage", 10);
    
    // Initialize subscriber
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_out", 10,
        std::bind(&MyAGVNode::cmdVelCallback, this, std::placeholders::_1));
    
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Setup serial port
    setupSerial();
    
    // Create timer for main loop
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0/SAMPLE_FREQ),
        std::bind(&MyAGVNode::timerCallback, this));
        
    RCLCPP_INFO(get_logger(), "MyAGV node initialized");
}

MyAGVNode::~MyAGVNode() {
    if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
    }
}

void MyAGVNode::setupSerial() {
    try {
        serial_port_ = std::make_unique<boost::asio::serial_port>(io_service_);
        serial_port_->open("/dev/ttyAMA2");
        serial_port_->set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(get_logger(), "Serial port error: %s", e.what());
        throw;
    }
}

void MyAGVNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_linear_x_ = msg->linear.x;
    cmd_linear_y_ = msg->linear.y;
    cmd_angular_z_ = msg->angular.z;
}

bool MyAGVNode::readSerialData() {
    try {
        uint8_t byte;
        // Wait for header
        while (true) {
            boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1));
            if (byte == header_[0]) {
                boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1));
                if (byte == header_[1]) break;
            }
        }

        // Read data
        std::vector<uint8_t> data(TOTAL_RECEIVE_SIZE);
        size_t bytes_read = boost::asio::read(*serial_port_, 
            boost::asio::buffer(data.data(), TOTAL_RECEIVE_SIZE));
            
        if (bytes_read != TOTAL_RECEIVE_SIZE) {
            RCLCPP_ERROR(get_logger(), "Incomplete read: %zu bytes", bytes_read);
            return false;
        }

        // Parse velocities
        vx_ = (static_cast<double>(data[0]) - 128.0) * 0.01;
        vy_ = (static_cast<double>(data[1]) - 128.0) * 0.01;
        vtheta_ = (static_cast<double>(data[2]) - 128.0) * 0.01;

        // Parse IMU data
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
        imu_msg->header.stamp = this->now();
        imu_msg->header.frame_id = "imu";

        // Accelerometer data
        imu_msg->linear_acceleration.x = ((data[3] + data[4] * 256) - 10000) * 0.001;
        imu_msg->linear_acceleration.y = ((data[5] + data[6] * 256) - 10000) * 0.001;
        imu_msg->linear_acceleration.z = ((data[7] + data[8] * 256) - 10000) * 0.001;

        // Gyroscope data
        double gyro_z = ((data[13] + data[14] * 256) - 10000) * 0.1;
        
        // Calibrate gyro if needed
        if (offset_count_ < OFFSET_COUNT) {
            gyro_offset_z_ += gyro_z;
            offset_count_++;
            if (offset_count_ == OFFSET_COUNT) {
                gyro_offset_z_ /= OFFSET_COUNT;
                RCLCPP_INFO(get_logger(), "IMU calibration complete. Z offset: %f", gyro_offset_z_);
            }
            return true;
        }

        // Apply calibration
        gyro_z -= gyro_offset_z_;
        imu_msg->angular_velocity.z = gyro_z;

        // Publish IMU data
        imu_pub_->publish(std::move(imu_msg));

        return true;
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(get_logger(), "Serial read error: %s", e.what());
        return false;
    }
}

void MyAGVNode::writeSpeed(double vx, double vy, double vtheta) {
    // Clamp values
    vx = std::clamp(vx, -1.0, 1.0);
    vy = std::clamp(vy, -1.0, 1.0);
    vtheta = std::clamp(vtheta, -1.0, 1.0);

    // Convert to bytes
    uint8_t x_send = static_cast<uint8_t>(vx * 100) + 128;
    uint8_t y_send = static_cast<uint8_t>(vy * 100) + 128;
    uint8_t rot_send = static_cast<uint8_t>(vtheta * 100) + 128;
    uint8_t checksum = (x_send + y_send + rot_send) & 0xFF;

    // Construct and send message
    std::vector<uint8_t> msg = {0xfe, 0xfe, x_send, y_send, rot_send, checksum};
    boost::asio::write(*serial_port_, boost::asio::buffer(msg));
}

void MyAGVNode::updateOdometry(const rclcpp::Duration & dt) {
    double dt_seconds = dt.seconds();
    
    // Update theta using calibrated IMU data
    theta_ += vtheta_ * dt_seconds;
    
    // Update position using wheel velocities and current orientation
    double delta_x = (vx_ * std::cos(theta_) - vy_ * std::sin(theta_)) * dt_seconds;
    double delta_y = (vx_ * std::sin(theta_) + vy_ * std::cos(theta_)) * dt_seconds;
    
    x_ += delta_x;
    y_ += delta_y;
}

void MyAGVNode::publishTransform() {
    auto t = geometry_msgs::msg::TransformStamped();
    t.header.stamp = this->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(t);
}

void MyAGVNode::publishOdometry() {
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.linear.y = vy_;
    odom.twist.twist.angular.z = vtheta_;
    
    odom_pub_->publish(odom);
}

void MyAGVNode::timerCallback() {
    if (readSerialData()) {
        auto current_time = this->now();
        auto dt = current_time - last_time_;
        
        updateOdometry(dt);
        publishTransform();
        publishOdometry();
        writeSpeed(cmd_linear_x_, cmd_linear_y_, cmd_angular_z_);
        
        last_time_ = current_time;
    }
}
