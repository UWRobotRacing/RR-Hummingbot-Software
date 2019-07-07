/** @file rr_topci_names.hpp
 *  @author Andrew Jin
 *  @author Waleed Ahmed (w29ahmed)
 *  @competition IARRC 2019
 *
 *  @brief Defines the topic and service names across the project
 */

#ifndef RR_TOPIC_NAMES
#define RR_TOPIC_NAMES

// Standard includes
#include <string>

static std::string rr_str = "/rr_bot/";
static std::string rr_cv_str = "rr_cv/";
static std::string rr_supervisor_str = "rr_supervisor/";
static std::string rr_path_planner_str = "rr_path_planner/";
static std::string rr_controller_str = "rr_controller/";

namespace rr_sensor_topics {
  static std::string zed_left = "/zed/zed_node/left/image_rect_color";
  static std::string zed_right = "/zed/zed_node/right/image_rect_color";
  static std::string zed_depth = "/zed/zed_node/depth/depth_registered";
}

namespace rr_cv {
  static std::string lane_detection_occupancy_grid = rr_str + rr_cv_str + "lane_detection/occupancy_grid";
}

namespace rr_supervisor {
  static std::string start_race_service = rr_str + rr_supervisor_str + "start_race_service";
  static std::string count_lap_service = rr_str + rr_supervisor_str + "count_lap_service";
  static std::string twist_cmd = rr_str + rr_supervisor_str + "twist_cmd";
  static std::string no_movement = rr_str + rr_supervisor_str + "no_movement";
}

namespace rr_controller {
  static std::string twist_cmd = rr_str + rr_controller_str + "twist_cmd";

}

namespace rr_mapper {

}

namespace rr_path_planner {
  static std::string twist_cmd = rr_str + rr_path_planner_str + "twist_cmd";
}



#endif // RR_TOPIC_NAMES