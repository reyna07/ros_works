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
    //inlined for that its less than 10 lines
    void clearData()
    {
        inputString.clear();
        newData = false;
        recWindow = "  ";
    }
    ros::Publisher p_pub;
    ros::Publisher a_pub;
    ros::Publisher u_pub;
    serial::Serial ser;

private:
    bool newData = false;
    std::string recWindow = "..";
    std::string inputString;
};
#endif
