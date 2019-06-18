/** @file laser_mapper.hpp
 *  @author Andrew Jin (D29Jin)
 *  @competition IARRC 2019
 */

#ifndef INTERFACE_H
#define INTERFACE_H

// ROS headers
#include <ros/ros.h>

#include <rr_interface/Transmitter.h>

/* 
  This is a flag that is sent by the transmitter (Jetson -> Coretex M4)
  It is designed to send specific commands to ensure that the robot does
  not deviate during certain cases
  All the values defined above are binary values and should always be the case
*/ 
enum class jetsonFlag : uint8_t {
  ESTOP = 1
  // Stuff
};

enum class coretexFlag : uint8_t {
  ALIVE = 0,
  STABLE = 1,
  CONNECTED = 2
};

class Interface
{
  public:
    Interface(ros::NodeHandle nh);
    ~Interface();

    struct Transmitter {
      int8_t steer_angle; // Degrees
      uint16_t speed; // cm/s
      jetsonFlag flag; // Flag enum
    };

    struct Receiver {
      coretexFlag flag; // Flag enum
    };

    std::vector<uint16_t> Serialize(std::vector<uint16_t> transmitter);
    Receiver Deserialize(char* buffer);

    Transmitter transmitter_;
    Receiver receiver_;
  private:

    // Subscribers/Advertisers
    ros::Subscriber transmitter_subscriber_;
    ros::Publisher receiver_publisher_;

    void TransmitterCallback(const rr_interface::Transmitter &msg);
};

#endif  // INTERFACE_H
