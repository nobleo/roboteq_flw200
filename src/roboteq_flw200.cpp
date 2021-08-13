// Copyright <2021>
#include <cmath>
#include <iostream>
#include <rcl_interfaces/msg/parameter.hpp>
#include <roboteq_flw200_node/roboteq_flw200.hpp>
#include <string>
#include <unistd.h>

FlowSensor::FlowSensor(const std::string &name, rclcpp::NodeOptions const &options)
    : Node(name, options),
      port_("/dev/ttyS2"),
      baud_(9600)
{
    create_parameter();
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("Optical_sensor_pose", 10);
    serial_ptr_ = std::make_shared<serial::Serial>(port_, baud_, serial::Timeout::simpleTimeout(1000));

    RCLCPP_INFO(this->get_logger(), "%s is open(%d)", port_.c_str(), serial_ptr_->isOpen());

    thread_ = std::thread{[this]() -> void
                          {
                              while (rclcpp::ok())
                              {
                                  parse();
                                  unsigned int miliseconds = 1000;
                                  usleep(100 * miliseconds);
                              }
                          }};
}

FlowSensor::~FlowSensor()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void FlowSensor::get_quaternions(float &x, float &y, float &z, float &w)
{
    char command[20];
    strcpy(command, "?QO\r"); //command to retrieve quaternions
    serial_ptr_->write(command);
    bool available = true;
    char val[20];
    char cToStr[2];
    cToStr[1] = '\0';
    int counter = 0;
    strcpy(val, "");
    try
    {
        while (available)
        {
            serial_ptr_->read(buffer_, 1);
            // some parsing to add the recieved values to the correct variable
            cToStr[0] = (char)buffer_[0];
            if (cToStr[0] == '=')
            {
                strcpy(val, "");
            }
            else if (cToStr[0] == ':')
            {
                switch (counter)
                {
                case 0:
                    x = std::stoi(val);
                    break;
                case 1:
                    y = std::stoi(val);
                    break;
                case 2:
                    z = std::stoi(val);
                    break;
                default:
                    throw std::invalid_argument("Looks like the received data is corrupt.");
                }
                counter++;
                strcpy(val, "");
            }
            else
            {
                strcat(val, cToStr);
            }
            available = serial_ptr_->available();
        }

        //the last part is the last variable
        w = std::stoi(val);
    }
    catch (const std::invalid_argument &ia)
    {
        printf("invalid argument: %s\n", ia.what());
    }
}

void FlowSensor::get_position(float &x, float &y)
{
    char command[20];
    strcpy(command, "?MM\r"); //command to retrieve the position in 10ths of a milimeter
    serial_ptr_->write(command);
    bool available = true;
    char val[20];
    char cToStr[2];
    cToStr[1] = '\0';
    strcpy(val, "");
    try
    {
        while (available)
        {
            serial_ptr_->read(buffer_, 1);
            // some parsing to add the recieved values to the correct variable
            cToStr[0] = (char)buffer_[0];

            // some parsing to add the recieved values to the correct variable
            if (cToStr[0] == '=')
            {
                strcpy(val, "");
            }
            else if (cToStr[0] == ':')
            {
                x = std::stoi(val) / 10000.0f; // devide by 10000 to get meters
                strcpy(val, "");
            }
            else
            {
                strcat(val, cToStr);
            }
            available = serial_ptr_->available();
        }

        //the last part is the y value
        y = std::stoi(val) / 10000.0f;
    }
    catch (const std::invalid_argument &ia)
    {
        printf("invalid argument: %s\n", ia.what());
    }
}

void FlowSensor::parse()
{
    auto pose = geometry_msgs::msg::Pose();
    float x, y, z, w;
    get_quaternions(x, y, z, w);
    pose.orientation.x = x;
    pose.orientation.y = y;
    pose.orientation.z = z;
    pose.orientation.w = w;
    float px, py;
    get_position(px, py);
    pose.position.x = px;
    pose.position.y = py;
    RCLCPP_DEBUG(this->get_logger(), "Publishing: 'Q:[%.1f %.1f %.1f %.1f] P:[%.4f %.4f]'", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, pose.position.x, pose.position.y);
    publisher_->publish(pose);
}

void FlowSensor::create_parameter()
{
    port_ = declare_parameter<std::string>("port", "/dev/ttyS2");
    baud_ = declare_parameter<int>("baud", 9600);

    set_on_parameters_set_callback(
        [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
        {
            auto result = rcl_interfaces::msg::SetParametersResult();
            result.successful = true;
            for (auto const &p : parameters)
            {
                result.successful &= handle_parameter(p);
            }
            return result;
        });
}

bool FlowSensor::handle_parameter(rclcpp::Parameter const &param)
{
    if (param.get_name() == "port")
    {
        port_ = param.as_string();
    }
    else if (param.get_name() == "baud")
    {
        baud_ = param.as_int();
    }
    else
    {
        return false;
    }

    return true;
}
