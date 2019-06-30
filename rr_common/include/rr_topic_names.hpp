/** @file rr_topci_names.hpp
 *  @author Andrew Jin
 *  @author Waleed Ahmed (w29ahmed)
 *  @competition IARRC 2019
 *
 *  @brief Defines the topic and service names across the project
 */

#ifndef RR_TOPIC_NAMES
#define RR_TOPIC_NAMES

#include <string.h>

static std::string rr_str = "/rr_bot/";
static std::string rr_cv_str = "rr_cv/";

namespace rr_sensor_topics {
    static std::string zed_left = "/zed/zed_node/left/image_rect_color";
    static std::string zed_right = "/zed/zed_node/right/image_rect_color";
    static std::string zed_depth = "/zed/zed_node/depth/depth_registered";
}

namespace rr_cv {
    static std::string lane_detection_occupancy_grid = rr_str + rr_cv_str + "lane_detection/occupancy_grid";
}

namespace rr_controller {

}

namespace rr_mapper {

}

namespace rr_path_planner {

}

namespace rr_supervisor {

}

#endif // RR_TOPIC_NAMES