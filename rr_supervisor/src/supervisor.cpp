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
 *    /arduino/battery_state
 *  @author Waleed Ahmed(w29ahmed) && Yuchi(Allan) Zhao
 *  @competition IARRC 2019
 */

#include <supervisor.hpp>

using namespace supervisor_topic;
using namespace arduino_topic;
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

    // Setup subscribers to monitor battery and speed
    cmd_sub_ = nh_.subscribe(vel_cmd, 1, &Supervisor::TrackSpeed, this);
    battery_sub_ = nh_.subscribe(battery_state, 1, &Supervisor::MonitorBattery, this);

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
        lap_count_+=1;
        res.success=true;
        ROS_INFO(¨Drag race complete! shutting down¨)
    }
    else if(race_type_==¨circuit¨)
    {
        if (lap_count_ < 3)
        {
            lap_count_ += 1;
            if (lap_count_ == 3)
            {
                res.success = true;
                ROS_INFO("Circuit race complete! shutting down");
            }
            else
            {
                res.success = false;
                ROS_INFO("Lap %d of %d complete!", lap_count_, 3);
            }
        }
    }

    //indicate successful run of the service 
    if (res.success)
    {
        this->FinishRace();
    }
    return true;
}
void Supervisor::TrackSpeed(const geometry_msgs::TwistConstPtr& msg)
{
    twist_msg_count_+=1;
    speed_sum_+=(pow(msg->linear.x, 2) + pow(msg->linear.y, 2))
}

void Supervisor::MonitorBattery(const std_msgs::Int8::ConstPtr& msg)
{
  if (msg->data <= 10)
  {
    ROS_INFO("Battery is less than 10 percent, ending race...");
    this->FinishRace();
  }
}

void Supervisor::FinishRace()
{
  // Calculate average speed and elapsed time since start of race
  average_speed_ = speed_sum_ / twist_msg_count_;
  race_time_ = float(clock() - begin_time_) / CLOCKS_PER_SEC;

  // Construct a timestamp with the following format:
  // Year_Month_day--Hours_minutes_seconds
  time_t rawtime;
  struct tm * time_info;
  char date_string[50];
  time(&rawtime);
  time_info = localtime(&rawtime);
  strftime(date_string, 50, "%Y_%m_%d---%H_%M_%S", time_info);

  // Open a file to write to
  std::ofstream race_metrics_file;
  std::ostringstream file_name;
  file_name << "~/" << race_type_ << "_" << date_string << ".txt";
  race_metrics_file.open(file_name.str().c_str());

  // Write to file
  std::ostringstream metrics;

  metrics << "Date: " << date_string << "\n";
  metrics << "Race type: " << race_type_ << "\n";
  metrics << "Race time: " << race_time_ << "\n";
  metrics << "Average speed: " << average_speed_ << "\n";

  race_metrics_file << metrics.str();
  race_metrics_file.close();

  // Publish null vector to twist multiplexer and lock out any other messages
  // from being published, which will stop the robot
  twist_pub_.publish(twist_msg_);
  this-> IdleRobot();
}

