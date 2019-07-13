
/** @file laser_mapper.hpp
 *  @author Ajay Kumar Singh
 *  @author Jungwook Lee
 *  @author Sirui Song
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <limits> // for infinity
#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <rr_computer_vision/TrafficSign.h>

class PathPlanner {
public:
    PathPlanner();
private:
    //functions
    void Init();
    void GetParams();
    //functions for path variables calculations
    std::vector< std::vector <double> > GetAnglesAndWeights(double max_angle, int num_paths);
    void GenerateIdealPaths();  //paths without obstacle
    void GenerateRealPaths(); //paths with obstacle in real time
    int xyToMapIndex(double x, double y);
    std::vector<geometry_msgs::Point> rayTrace(double x0, double y0, double x1, double y1);
    void ProcessMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void EnableCallBack(const std_msgs::Int8::ConstPtr& msg);
    void SignDirectionCallback(const rr_computer_vision::TrafficSign& msg);
    bool IsCellOccupied(int index);
    int CheckLength(int angle_index);
    double Velocity(double dist, double steer);
    double StopDistFromVel(geometry_msgs::Twist velocity);
    void Accumulate(double speed);

    //ROS nodes, pub, sub, msgs & variables
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber traffic_state_sub_;

    double wheel_to_wheel_dist_;
    ros::Publisher cmd_pub_;
    nav_msgs::OccupancyGrid::ConstPtr map_;  //map in 2d grid
    geometry_msgs::Twist vel_cmd_;

    ros::Subscriber enable_sub;
    std_msgs::Int8 enable;

    //map parameters
    int map_W_;
    int map_H_;
    double resolution_;

    //Car parameters
    int X_START_;
    int Y_START_;
    double CAR_WIDTH_;

    // Trajectory Rollout Path variables
    std::vector< std::vector <double> > angles_and_weights;   //It is Nx2 matrix; 1st column is angle, 2nd column is weight
    std::vector< std::vector <double> > path_distance;
    std::vector< std::vector < std::vector <int> > > trajectory;
    int NUM_PATHS_;
    double PLANNER_VELOCITY_;
    int TRAJECTORY_STEPS_;
    int OBS_THRESHOLD_;
    double ANGLE_LIMIT_;
    double MIN_DISTANCE_;
    double dt_;
    double HORIZONTAL_LINE_RATIO_;
    int HORIZONTAL_LINE_THICKENESS_;

    // Desired Path Setup
    double ANGLE_WEIGHT_;
    double DISTANCE_WEIGHT_;
    double ANGLE_WEIGHT_DIFF_;
    double MAX_STEERING_ANGLE_;

    // Desired Velocity Setup
    double UX_DESIRED_;
    double ANGLE_VEL_DAMP_;
    double MIN_ACCELERATION_ANGLE_;
    double STRAIGHT_SPEED_;
    
    // Offset box params, obs in the box is ignored
    double min_offset_dist_;
    double MIN_STOPPING_DIST_;
    double STOPPING_FACTOR_;
    double DIST_REWARD_FACTOR_;

    // Left Right Determiner
    uint8_t prev_sign_choice_;
    double distSinceLastTurn_;
    double prev_speed_;
    double prev_time_;
    double maxTurnLength_;
    
    //Debug variables
    bool VISUALIZATION_;
    bool DEBUG_ON_;
    void DrawPath(ros::Publisher& pub, visualization_msgs::Marker& points,int id, int index, int R, int G, int B, float scale, float alpha);
    int all_path_marker_id_;  //used as an id for drawing path using visualization_msgs
    int selected_path_marker_id_;
    ros::Publisher all_path_pub_;
    ros::Publisher selected_path_pub_;
    std::vector <visualization_msgs::Marker> trajectory_marker_vector_;  //used for visualizing selected_path
    visualization_msgs::Marker trajectory_points_;
    //drawing axis
    visualization_msgs::Marker X_axis_marker_points;
    visualization_msgs::Marker Y_axis_marker_points;
    ros::Publisher X_axis_pub;
    ros::Publisher Y_axis_pub;
    int X_axis_marker_id;
    int Y_axis_marker_id;
    //drawing rayTrace
    std::vector <visualization_msgs::Marker> trajectory_marker_rayTrace_;
    ros::Publisher rayTrace_pub_;
    int trajectory_marker_rayTrace_id_;
};

#endif