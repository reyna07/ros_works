/**
 * @file: parse_serial.cpp
 * @brief: Parse data provided through UART serial stream into 3 groups and publish
 * @author: yemreikiz
 * */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <parse_serial/K_Personel.h>
#include <parse_serial/K_Asset.h>
#include <sstream>

serial::Serial ser;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parse_serial_node");
    ros::NodeHandle n;

    ros::Publisher p_pub = n.advertise<parse_serial::K_Personel>("personel", 1);

    ros::Publisher a_pub = n.advertise<parse_serial::K_Asset>("asset", 1);

    ros::Publisher u_pub = n.advertise<std_msgs::String>("update", 1);

    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(9600);
        serial::Timeout tout = serial::Timeout::simpleTimeout(1000); //timeout in ms
        ser.setTimeout(tout);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        ros::spinOnce();

        if (ser.available())
        {
            ROS_INFO_STREAM("Reading from serial port");
            std::string serInput;
            serInput = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << serInput.data());
            //serialParser(serInput,0,p_pub, a_pub, u_pub);
        }
    }
    return 0;
}

/**
 * @brief: Parses an input string based on $$your_message,elementA,elementB** convention
 * @param: Input string passed by reference.
 * */
void serialParser(const std::string &input, int start_pos, ros::Publisher& p_pub, ros::Publisher& a_pub, ros::Publisher& u_pub)
{
    int pos = input.find("$$");
    std::string field;

    if (pos != std::string::npos)
    {
        pos += 2; //actual position of data
        while (input[pos] != ',')
        {
            field += input[pos++];
        }
        if (field == "K_Personel")
        {
            parse_serial::K_Personel new_personel;
            std::string temp;
            //get name
            while (input[pos] != ',' && pos < input.size() - 1)   new_personel.name += input[pos++];
            //get age
            while (input[pos] != ',' && pos < input.size() - 1)   temp += input[pos++];
            new_personel.age = std::stoi(temp);
            temp.clear();
            //get weight
            while (input[pos] != ',' && pos < input.size() - 1)   temp += input[pos++];
            new_personel.weight = std::stof(temp);
            temp.clear();
            //get height
            while (input[pos] != '*' && pos < input.size() - 1)   temp += input[pos++];
            new_personel.height = std::stoi(temp);
            
            if(input[pos] != '*')//data package is not complete
            {
                /* code */
            }
            
            pos+=2;         //beginning of the next package

            /*Publish output before leaving*/
            p_pub.publish(new_personel);

            if (pos == input.size() - 1)    //end of input stream, exit case
            {
                return;
            }
            else       //finish input recursively
            {
                serialParser(input, pos, p_pub, a_pub, u_pub);
            }
            
        }
    }
}