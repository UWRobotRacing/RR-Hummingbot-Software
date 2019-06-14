/** @file laser_mapper.hpp
 *  @author Andrew Jin (D29Jin)
 *  @competition IARRC 2019
 */

#ifndef LASERMAPPER_H
#define LASERMAPPER_H

// ROS headers
#include <ros/ros.h>

// ROS Messages 
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>



/*
 * Callback Class for laser to Occumpancy Grid Format/OpenCV Format
 */

class LaserMapper
{
  public:
    LaserMapper(ros::NodeHandle nh);
    ~LaserMapper();

    void PublishMap();

  private:

    void InitializeSubscribersandPublishers();

    // Subscribers
    void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void LaneDetectionCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // Methods
    void InitMap();
    void GetParam();
    void UpdateLaserMap(const int& x, const int& y, const double& value);
    double CheckMap(const int& x, const int& y);
    void RayTracing(const float& angle, const float& range, const int& inflate_factor);
    std::vector<int> ShiftMap(std::vector<int> prev_map);
    std::vector<int> RotateMap(std::vector<int> curr_map, double new_ang); 
    

    // ROS Variables
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber lane_detection_sub_;
    tf::TransformListener position_listener_;

    // Map Variables
    std::vector<int> belief_map_;
    nav_msgs::OccupancyGrid lane_detection_msg_;
    sensor_msgs::LaserScan lidar_msg_;

    std_msgs::Header prev_header_;

    // Map Parameters
    double map_res_;
    double map_orientation_;
    int map_width_;
    int map_height_;

    // Scan Parameters
    double max_angle_;
    double min_angle_;
    double min_range_;
    double max_range_;
    int samplerate_;
    int inflate_obstacle_;
    int scan_subsample_;
    double LASER_ORIENTATION_;

    // Map Values
    const double OBS_SCALE_ = 1;

    // lane_detection Map Values
    int offset_height_left_;
    int offset_height_right_;
    int offset_width_left_;
    int offset_width_right_;

    enum CellState {
    NO_OBS_ = 0,
    OBS_ = 100,
    UNKNOWN_ = -1
    };

    //Storing values
    double prev_x_;
    double prev_y_;
    double prev_ang_;
};

#endif  // LASERMAPPER_H
