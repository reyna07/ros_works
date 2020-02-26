#ifndef __PARSE_SERIAL_H__
#define __PARSE_SERIAL_H__
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <parse_serial/K_Personel.h>
#include <parse_serial/K_Asset.h>
#include <sstream>


class parseSerial{
public:
    parseSerial();
    parseSerial(ros::NodeHandle& n);
    void serialParser();
    bool serialReadwMarkers();
    ros::Publisher p_pub;
    ros::Publisher a_pub;
    ros::Publisher u_pub;
    serial::Serial ser;
    bool newData = false;
    std::string inputString;
    std::string recWindow = "..";

};
#endif
