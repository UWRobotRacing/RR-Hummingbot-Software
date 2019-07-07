/** @file supervisor.hpp
 *  @brief Supervisor prototypes
 *  @author Waleed Ahmed(w29ahmed)
 *  @author Allan Zhao
 *  @competition IARRC 2019
 */

 // Include guard
#ifndef SUPERVISOR_H
#define SUPERVISOR_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

class Supervisor
{
  public:
    Supervisor();
    ~Supervisor();

    bool race_started;

  private:
    // ROS variables
    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    ros::Publisher null_lock_;
    ros::ServiceServer start_race_service_;
    ros::ServiceServer count_lap_service_;

    // Ros messages
    std_msgs::Bool bool_msg_;
    geometry_msgs::Twist twist_msg_;
    geometry_msgs::Vector3 null_vector_;
    
    // Internal variables
    int lap_counter_;
    int lap_count_;

    // Service callback methods
    bool StartRace(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool CountLap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void IdleRobot();
    void FinishRace();
};

#endif  // SUPERVISOR_H

