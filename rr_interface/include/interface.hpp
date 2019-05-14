/** @file laser_mapper.hpp
 *  @author Andrew Jin (D29Jin)
 *  @competition IARRC 2019
 */

#ifndef INTERFACE_H
#define INTERFACE_H

// ROS headers
#include <ros/ros.h>

#include <rr_interface/Transmitter.h>

class Interface
{
  public:
    Interface(ros::NodeHandle nh);
    ~Interface();

    std::vector<uint16_t> Serialize(std::vector<uint16_t> transmitter);
    std::vector<uint16_t> Deserialize(std::vector<uint16_t> receiver);

    struct Transmitter {
      uint8_t speed;
      uint8_t steer;
      uint8_t position;
    };

    struct Receiver {
      char payload;
    };

    Transmitter transmitter_;
    Receiver receiver_;
  private:

    // Subscribers/Advertisers
    ros::Subscriber transmitter_subscriber_;
    ros::Publisher receiver_publisher_;

    void TransmitterCallback(const rr_interface::Transmitter &msg);
};

#endif  // INTERFACE_H
