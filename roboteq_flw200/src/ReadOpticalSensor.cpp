#include <cstdio>

// Read Optical Sensor.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <sstream>
#include "ceSerial.h"
#include <cstring>

using namespace std;
using namespace ce;
using namespace std::chrono_literals;

class Sensor_reader
{
	public:
	Sensor_reader():
		m_com("/dev/ttyS2", 9600, 8, 'N', 1) // Linux serial port
	{
		printf("Opening port %s.\n", m_com.GetPort().c_str());
		if (m_com.Open() == 0) {
			printf("OK.\n");
		}
		else {
			printf("Error.\n");
		}
	}

	~Sensor_reader()
	{
		printf("Closing port %s.\n", m_com.GetPort().c_str());
		m_com.Close();
	}

	void get_quaternions(float& x, float& y, float& z, float& w)
	{
		char command[20];
		strcpy(command, "?QO\r"); //command to retrieve quaternions
		bool successFlag = m_com.Write(command);
		ceSerial::Delay(5); // delay to wait for a response
		char val[20];
		char cToStr[2];
		cToStr[1] = '\0';
		int counter = 0;
		strcpy(val, "");
		while (successFlag)
		{
			char c = m_com.ReadChar(successFlag); // read a char
			cToStr[0] = c;
			// some parsing to add the recieved values to the correct variable
			if (c == '=')
			{
				strcpy(val, "");
			}
			else if (c == ':')
			{
				switch (counter)
				{
				case 0:
					x = stoi(val);
					break;
				case 1:
					y = stoi(val);
					break;
				case 2:
					z = stoi(val);
					break;
				default:
					printf("error\n");
				}
				counter++;
				strcpy(val, "");
			}
			else
			{
				strcat(val, cToStr);
			}
		}

		//the last part is the last variable
		w = stoi(val);
	}

	void get_position(float& x, float& y)
	{
		char command[20];
		strcpy(command, "?MM\r");//command to retrieve the position in 10ths of a milimeter
		bool successFlag = m_com.Write(command);
		ceSerial::Delay(5); // delay to wait for a response
		char val[20];
		char cToStr[2];
		cToStr[1] = '\0';
		strcpy(val, "");
		while (successFlag)
		{
			char c = m_com.ReadChar(successFlag); // read a char
			cToStr[0] = c;
			// some parsing to add the recieved values to the correct variable
			if (c == '=')
			{
				strcpy(val, "");
			}
			else if (c == ':')
			{
				x = stoi(val) / 10000.0f; // devide by 10000 to get meters
				strcpy(val, "");
			}
			else
			{
				strcat(val, cToStr);
			}
		}

		//the last part is the y value
		y = stoi(val) / 10000.0f;
	}

	ceSerial m_com;
};

	class MinimalPublisher : public rclcpp::Node
	{
	public:
		MinimalPublisher()
			: Node("optical_sensor_publisher"), count_(0)
		{
			m_readerPtr = new Sensor_reader();
			publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("Optical_sensor_pose", 10);
			timer_ = this->create_wall_timer(
				100ms, std::bind(&MinimalPublisher::timer_callback, this));
		}

	private:
		void timer_callback()
		{
			auto pose = geometry_msgs::msg::Pose();
			float x,y,z,w;
			m_readerPtr->get_quaternions(x,y,z,w);
			pose.orientation.x = x;
			pose.orientation.y = y;
			pose.orientation.z = z;
			pose.orientation.w = w;
			float px,py;
			m_readerPtr->get_position(px,py);
			pose.position.x = px;
			pose.position.y = py;
			RCLCPP_DEBUG(this->get_logger(), "Publishing: 'Q:[%.1f %.1f %.1f %.1f] P:[%.4f %.4f]'", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, pose.position.x, pose.position.y);
			publisher_->publish(pose);
		}

		Sensor_reader* m_readerPtr;
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
		size_t count_;
	};

	int main(int argc, char* argv[])
	{
		//ros::init(argc, argv, "optical_sensor_publisher");

		 // Initialize ROS 2
		rclcpp::init(argc, argv);

		// Start processing data from the node as well as the callbacks and the timer
		rclcpp::spin(std::make_shared<MinimalPublisher>());

		// Shutdown the node when finished
		rclcpp::shutdown();
		return 0;
	}


