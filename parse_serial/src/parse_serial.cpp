/**
 * @file: parse_serial.cpp
 * @brief: Parse data provided through UART serial stream into 3 groups and publish
 * @author: yemreikiz
 * */
#include "parse_serial.h"

/**
 * @brief: parseSerial constructor
 * */
parseSerial::parseSerial(ros::NodeHandle &n)
{
  p_pub = n.advertise<parse_serial::K_Personel>("personel", 1);

  a_pub = n.advertise<parse_serial::K_Asset>("asset", 1);

  u_pub = n.advertise<std_msgs::String>("update", 1);

  try
  {
    ser.setPort("/dev/ttyS19");
    ser.setBaudrate(9600);
    serial::Timeout tout = serial::Timeout::simpleTimeout(1000); //timeout in ms
    ser.setTimeout(tout);
    ser.open();
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return;
  }
  if (ser.isOpen())
  {
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
    ROS_ERROR_STREAM("Unable to open port ");
  }
}
/**
 * @brief: Parses an input string based on $$your_message,elementA,elementB** convention
 * @param: Input string passed by reference.
 * */
void parseSerial::serialParser()
{
  std::string field;
  int pos = 0; //iterator
  while (inputString[pos] != ',' && pos < (inputString.size() - 1) )
  {
    field += inputString[pos++];
  }
  pos++;
  if (field.compare("K_Personel") == 0 )
  {
    parse_serial::K_Personel new_personel;
    std::string temp;
    //get name
    while (inputString[pos] != ',' && pos < inputString.size() - 1)
      new_personel.name += inputString[pos++];
      pos++;
    //get age
    while (inputString[pos] != ',' && pos < inputString.size() - 1)
      temp += inputString[pos++];
      pos++;
    new_personel.age = std::stoi(temp);
    temp.clear();
    //get weight
    while (inputString[pos] != ',' && pos < inputString.size() - 1)
      temp += inputString[pos++];
      pos++;
    new_personel.weight = std::stof(temp);
    temp.clear();
    //get height
    while (pos <= inputString.size() - 1)
      temp += inputString[pos++];
    new_personel.height = std::stoi(temp);

    /*Publish output before leaving*/
    p_pub.publish(new_personel);
  }
  else if (field.compare("K_Asset") == 0)
  {
    parse_serial::K_Asset new_asset;
    std::string temp = "";
    //get ID
    while (inputString[pos] != ',' && pos < inputString.size() - 1)
      temp += inputString[pos++];
    pos++;
    new_asset.assetID = std::stoi(temp);
    temp.clear();
    //get location
    while (inputString[pos] != ',' && pos < inputString.size() - 1)
      temp += inputString[pos++];
    new_asset.location = temp;
    temp.clear();

    /*Publish output before leaving*/
    a_pub.publish(new_asset);
  }
  else if (field.compare("K_Update") == 0)
  {
    std::string data = "Update";
    std_msgs::String msg;
    msg.data = data;
    u_pub.publish(msg);
  }
  else
  {
    ROS_INFO("'%s'", field.c_str());
    ROS_INFO("Received message with proper format but unrecognized field type.");
  }
}

/**
 * @brief: Captures an input string based on $$your_message,elementA,elementB** convention.
 * */
bool parseSerial::serialReadwMarkers()
{
  static bool recvInProgress = false;
  static uint8_t ndx = 0;
  std::string startS = "$$";
  std::string endS = "**";
  std::string rc;
  while (ser.available() > 0 && newData == false)
  {
    rc = ser.read(1);
    recWindow[0] = recWindow[1];
    recWindow[1] = rc[0];
    //ROS_INFO("recWindow: %s",recWindow.c_str());
    if (recvInProgress == true)
    {
      if (recWindow.compare(endS) != 0)
      {
        inputString += rc;
        ndx++;
        if (ndx >= 1024)
        {
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      }
      else
      {
        inputString[inputString.length() - 1] = '\0'; //remove the excess endmarkers recorded in the inputString

        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (recWindow.compare(startS) == 0)
    {
      recvInProgress = true;
    }
  }
  return newData;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "parse_serial_node");
  ros::NodeHandle n;
  parseSerial ps(n);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    ros::spinOnce();
    if(ps.serialReadwMarkers())
    {
      ps.serialParser();
      ps.newData = false;
      ps.inputString.clear();
      ps.recWindow = "  ";
    }
  }
  return 0;
}