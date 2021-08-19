# roboteq_flw200

A ros2 package to read sensor data from the roboteq flw200 optical sensor

First a serial communication is opened. Next every 100ms a command is send to the sensor to retrieve the x,y data and the quaternion orientation.
This is published in odom topic.
