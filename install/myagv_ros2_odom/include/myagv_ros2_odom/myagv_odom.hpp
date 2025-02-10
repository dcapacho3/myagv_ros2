// myagv.hpp
#ifndef MYAGV_NODE_HPP_
#define MYAGV_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/asio.hpp>
#include <Eigen/Dense>
#include <array>

class MyAGVNode : public rclcpp::Node {
public:
    MyAGVNode();
    virtual ~MyAGVNode();

private:
    // Constants
    static constexpr int TOTAL_RECEIVE_SIZE = 18;
    static constexpr int OFFSET_COUNT = 100;
    static constexpr double SAMPLE_FREQ = 100.0;  // Hz
    
    // Serial communication
    boost::asio::io_service io_service_;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::array<uint8_t, 2> header_{0xfe, 0xfe};
    
    // State variables
    double x_{0.0}, y_{0.0}, theta_{0.0};
    double vx_{0.0}, vy_{0.0}, vtheta_{0.0};
    double cmd_linear_x_{0.0}, cmd_linear_y_{0.0}, cmd_angular_z_{0.0};
    
    // IMU calibration
    double gyro_offset_z_{0.0};
    int offset_count_{0};
    
    // Timing
    rclcpp::Time last_time_;
    
    // ROS publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Methods
    void setupSerial();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    bool readSerialData();
    void writeSpeed(double vx, double vy, double vtheta);
    void updateOdometry(const rclcpp::Duration & dt);
    void publishTransform();
    void publishOdometry();
    void publishVoltage();
    void timerCallback();
};

#endif  // MYAGV_NODE_HPP_
