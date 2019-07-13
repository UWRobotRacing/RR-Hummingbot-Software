/** @file laser_mapper.hpp
 *  @author Andrew Jin (D29Jin)
 *  @competition IARRC 2019
 */

#ifndef INTERFACE_H
#define INTERFACE_H

// ROS headers
#include <ros/ros.h>

#include <rr_interface/Transmitter.h>

typedef struct
{
   int16_t jetson_ang;
   int16_t  jetson_spd;
   uint16_t jetson_flag;
}jetson_data_t;

typedef struct
{
   uint8_t        startByte;
   jetson_data_t  data;
   uint8_t        endByte;
}jetson_packet_t;

typedef union
{
  jetson_packet_t myFrame;
  uint8_t         serializedArray[8];
}jetson_union_t;

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
      jetson_union_t jetson;
    };
    /*
    struct Transmitter {
      int16_t steer_angle; // Degrees
      uint16_t speed; // cm/s
      uint16_t flag; // Flag enum
      uint16_t padding;
    };
     */

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
