/** @file supervisor.cpp
 *  @brief Supervisor class implementation
 *  @author Waleed Ahmed (w29ahmed)
 *  @author Yuchi(Allan) Zhao
 *  @competition IARRC 2019
 */

#include "supervisor.hpp"
#include "rr_topic_names.hpp"

/**
 * @brief Supervisor class constructor
 */
Supervisor::Supervisor() {
  race_started = false;

  null_vector_.x = 0.0; null_vector_.y = 0.0; null_vector_.z = 0.0;
  twist_msg_.linear = null_vector_;
  twist_msg_.angular = null_vector_;

  // Setup publishers for twist multiplexer
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>(rr_supervisor::twist_cmd, 1);
  null_lock_ = nh_.advertise<std_msgs::Bool>(rr_supervisor::no_movement, 1);

  // Setup service servers
  start_race_service_ = nh_.advertiseService(rr_supervisor::start_race_service, &Supervisor::StartRace, this);
  count_lap_service_  = nh_.advertiseService(rr_supervisor::count_lap_service, &Supervisor::CountLap, this);
}

Supervisor::~Supervisor() {

}

bool Supervisor::StartRace(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  race_started = true;

  // Set null_lock as false. Will allow path planner through
  bool_msg_.data=false;
  null_lock_.publish(bool_msg_);

  ROS_INFO("Race started!");
  return true;
}

bool Supervisor::CountLap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  // Increment lap count
  lap_counter_++;
  ROS_INFO("Lap %d of %d complete!", lap_counter_, lap_count_);

  // Check to see if we are at the end of the race
  if (lap_counter_ == lap_count_) {
    FinishRace();
    return true;
  }
  else {
    return false;
  }
}

void Supervisor::IdleRobot() {
  //Set null_lock as true. Will NOT allow path planner messages through
  bool_msg_.data = true;
  null_lock_.publish(bool_msg_);
}

void Supervisor::FinishRace() {
  // Publish null vector to twist multiplexer and lock out any other messages
  // from being published, which will stop the robot
  twist_pub_.publish(twist_msg_);
  IdleRobot();
}