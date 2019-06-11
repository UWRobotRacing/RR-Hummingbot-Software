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


// void Interface::TransmitterCallback(const rr_interface::Transmitter &msg) {
//     transmitter_.speed = msg.speed;
//     transmitter_.steer = msg.steer;
//     transmitter_.position = msg.position;
// }

Interface::Receiver Interface::Deserialize(char* buffer)
{
    Interface::Receiver rReceiver;

    // Converts the data to the correct struct
    Receiver *fromChar = (Receiver*)buffer;
    rReceiver.butt = fromChar->butt;
    rReceiver.butter = fromChar->butter;

    // ensures no dangling
    fromChar = nullptr;

    return rReceiver;
}

void Interface::TransmitterCallback(const rr_interface::Transmitter &msg) {
    transmitter_.butt = msg.butt;
    transmitter_.butter = msg.butter;
}