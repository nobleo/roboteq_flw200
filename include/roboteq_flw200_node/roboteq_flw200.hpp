// Copyright <2021>

#pragma once
#include "geometry_msgs/msg/pose.hpp"
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

class FlowSensor : public rclcpp::Node
{
public:
    FlowSensor(const std::string &name, rclcpp::NodeOptions const &options);
    ~FlowSensor();

private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
    std::shared_ptr<serial::Serial> serial_ptr_;
    std::string port_;
    uint baud_;
    uint8_t buffer_[2];
    std::thread thread_;

    void get_quaternions(float &x, float &y, float &z, float &w);
    void get_position(float &x, float &y);
    void parse();
    void create_parameter();
    bool handle_parameter(rclcpp::Parameter const &param);
};
