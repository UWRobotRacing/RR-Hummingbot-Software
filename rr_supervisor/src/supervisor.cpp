/** @file supervisor.cpp
 *  @brief Supervisor class implementation
 * 
 *  *  Services provided:
 *    /Supervisor/start_race
 *    /Supervisor/count_lap
 *
 *  Topics Published:
 *    /Supervisor/cmd
 *    /Supervisor/no_movement
 *    /Supervisor/enable_movement
 *
 *  Topics Subscribed:
 *    /rr_vehicle/vel_cmd
 *  @author Waleed Ahmed(w29ahmed) && Yuchi(Allan) Zhao
 *  @competition IARRC 2019
 */

#include <supervisor.hpp>

using namespace supervisor_topic;
using namespace rr_cmd_topics;

/**
 * @brief Supervisor class constructor
 */
Supervisor::Supervisor() 
{
    raceStarted=false;
    twist_msg_count_ = 0;

    null_vector_.x = 0.0; null_vector_.y = 0.0; null_vector_.z = 0.0;
    twist_msg_.linear = null_vector_;
    twist_msg_.angular = null_vector_;

      // Setup publishers for twist multiplexer
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd, 1);
    null_lock_ = nh_.advertise<std_msgs::Bool>(no_movement, 1);

    // Setup service servers
    start_race_service_ = nh_.advertiseService(start_race, &Supervisor::StartRace, this);
    count_lap_service_  = nh_.advertiseService(count_lap, &Supervisor::CountLap, this);

}

Supervisor::~Supervisor() 
{
    raceStarted=false;
    twist_msg_count_ = 0;
    null_vector_.x = 0.0; null_vector_.y = 0.0; null_vector_.z = 0.0;
    twist_msg_.linear = null_vector_;
    twist_msg_.angular = null_vector_;
}

// other methods
void Supervisor::IdleRobot()
{
    ROS_INFO(¨Robot Idle!¨);
    //Set null_lock as true. Will NOT allow path planner and joy messages through
    bool_msg_.data=true;
    null_lock_.publish(bool_msg_);
}

// Callback methods
bool Supervisor::StartRace(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    raceStarted=true;
    //Set null_lock as false. Will allow path planner and joy messages through
    bool_msg_.data=false;
    null_lock_.publish(bool_msg_);

    begin_time =clock();
    ROS_INFO(¨Race started!¨);
    return true;
}

bool Supervisor::CountLap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if(race_type==¨drag¨)
    {
        res.success=true;
        ROS_INFO(¨Drag race complete! shutting down¨)
    }
    else if(race_type_==¨circuit¨)
    {
        if (lap_count_ < 1)
        {
            lap_count_ += 1;
            if (lap_count_ == 1)
            {
                res.success = true;
                ROS_INFO("Circuit race complete! shutting down");
            }
            // else
            // {
            //     res.success = false;
            //     ROS_INFO("Lap %d of %d complete!", lap_count_, 3);
            // }
        }
    }

    //indicate successful run of the service 
    if (res.success)
    {
        this->FinishRace();
    }
    return true;
}

void Supervisor::FinishRace()
{
  // Publish null vector to twist multiplexer and lock out any other messages
  // from being published, which will stop the robot
  twist_pub_.publish(twist_msg_);
  this-> IdleRobot();
}

