/** @file laser_mapper.hpp
 *  @author Andrew Jin (D29Jin)
 *  @competition IARRC 2019
 */

#ifndef INTERFACE_H
#define INTERFACE_H

// ROS headers
#include <ros/ros.h>

class Interface
{
  private:
    struct Transmitter {
      uint8_t speed;
      uint8_t steer;
      uint8_t position;
    };

    struct Receiver {
    };

  public:
    Interface(ros::NodeHandle nh);
    ~Interface();

    std::vector<uint16_t> Serialize(std::vector<uint16_t> transmitter);
    std::vector<uint16_t> Deserialize(std::vector<uint16_t> receiver);

    Transmitter transmitter_;
    Receiver receiver_;
  private:
    // std::vector<uint16_t> transmitter_ {3,0};
    // std::vector<uint16_t> receiver_ {3,0};
    // uint32_t serialized_transmitter_;

};

#endif  // INTERFACE_H
