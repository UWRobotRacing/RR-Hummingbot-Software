/** @file supervisor.hpp
 *  @brief Supervisor prototypes
 *  @author Waleed Ahmed(w29ahmed) && Allan Zhao
 *  @competition IARRC 2019
 */

 // Include guard
#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include ¨msg_srv_names.hpp¨

class Supervisor
{
  public:
    //constructor & distructor
    Supervisor();
    Supervisor::~Supervisor();

    // Callback methods
    bool StartRace(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool CountLap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void FinishRace();
    
    // other methods & Variables
    void IdleRobot();
    bool raceStarted;

  private:
    // ROS Variables
    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    ros::Publisher null_lock_;
    ros::ServiceServer start_race_service_;
    ros::ServiceServer count_lap_service_;

    // Variables
    std_msgs::Bool bool_msg_;
    geometry_msgs::Twist twist_msg_;
    geometry_msgs::Vector3 null_vector_;
    
    int lap_count_;
    int twist_msg_count_;
};

#endif  // SUPERVISOR_H

