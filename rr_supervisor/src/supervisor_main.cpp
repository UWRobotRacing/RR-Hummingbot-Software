/** @file supervisor_main.cpp
 *  @brief Supervisor main method, initialize supervisor node
 *  @author Waleed Ahmed(w29ahmed)
 *  @author Yuchi(ALlan) Zhao
 *  @competition IARRC 2019
 */

#include "supervisor.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "supervisor");
  ROS_INFO("Initializing Supervisor node");
  ros::NodeHandle nh_;

  // Instantiate Supervisor object
  Supervisor supervisor;

  ROS_INFO("Robot Idle, waiting for traffic light...");

  ros::Rate r(5);
  while (ros::ok() && (!supervisor.race_started))
  {
    supervisor.IdleRobot();
    ros::spinOnce();
    r.sleep();
  }

  ros::spin();
  return 0;
}
