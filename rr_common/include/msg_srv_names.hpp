/** @file msg_srv_names.hpp
 *  @author Andrew Jin && Allan Zhao
 *  @competition IARRC 2019
 *
 *  @brief defines the topic and service names across the project
 */

#ifndef MSG_SRV_NAMES
#define MSG_SRV_NAMES

#include <string.h>

static std::string rr= "/rr_vehicle/";
static std::string supv= ¨/Supervisor/¨;
static std::string ard= ¨/arduino/¨;

namespace supervisor_topic{
    static std::string  cmd= supv + "cmd";
    static std::string  no_movement= supv + ¨no_movement¨;
    static std::string  count_lap= supv + ¨count_lap¨;
    static std::string  start_race= supv + ¨start_race¨;
}
namespace arduino_topic{
     static std::string  battery_state= ard + ¨battery_state¨;
}


namespace rr_sensor_topics{
}

namespace rr_cmd_topics{
    static std::string  vel_cmd= rr + ¨vel_cmd¨;
}
 
namespace rr_signal_srvs{
}

namespace rr_processed_topics{
}

#endif // MSG_SRV_NAMES