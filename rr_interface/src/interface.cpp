/** @file interface.cpp
 *  @author Andrew Jin (DongJunJin)
 *  @competition IARRC 2019
 */

// Local
#include "interface.hpp"

/**
 * @name Interface
 * @brief initiliazes the Interface class
 * @return NONE
 */
Interface::Interface(ros::NodeHandle nh) {
    transmitter_subscriber_ = nh.subscribe("interface/transmitter", 0 , &Interface::TransmitterCallback, this);
}

/**
 * @name ~Interface
 * @brief destructs the Interface class
 * @return NONE
 */
Interface::~Interface() {
}

Interface::Receiver Interface::Deserialize(char* buffer)
{
    if (buffer)
    {
        Interface::Receiver rReceiver;

        // Converts the data to the correct struct
        Receiver *fromChar = (Receiver*)buffer;
        return rReceiver;
    }
}

void Interface::TransmitterCallback(const rr_interface::Transmitter &msg) {
    switch(msg.flag) {
        case 0:
            transmitter_.flag = jetsonFlag::ESTOP;
        break;
    }
    transmitter_.speed = msg.speed;
    transmitter_.steer_angle = msg.steer_angle;
}