/** @file laser_mapper.cpp
 *  @author Jungwook Lee
 *  @author Andrew Jin (DongJunJin)
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

// Local
#include "laser_mapper.hpp"

/**
 * @name LaserMapper
 * @brief initiliazes the LaserMapper class
 * @return NONE
 */
LaserMapper::LaserMapper(ros::NodeHandle nh) {
    GetParam();
    InitializeSubscribersandPublishers();
}

/**
 * @name ~LaserMapper
 * @brief destructs the LaserMapper class
 * @return NONE
 */
LaserMapper::~LaserMapper() {
    belief_map_.clear();
}

/**
 * @name PublishMap
 * @brief Joins the entire map together
 *        & Publishes the full_map
 * @return NONE
 */
void LaserMapper::PublishMap() {
    nav_msgs::OccupancyGrid full_map;

    //Checks for lidar msg
    int n = floor(abs(min_angle_-lidar_msg_.angle_min)/lidar_msg_.angle_increment);
    double increment = lidar_msg_.angle_increment;

    if (prev_header_.seq != lidar_msg_.header.seq)
    {
    for (double i = min_angle_; i < max_angle_; i+= increment)
    {
        // Check for NaN ranges
        if (std::isnan (lidar_msg_.ranges[n]) == false)
        {
          DrawLine(i, lidar_msg_.ranges[n], inflate_obstacle_);
        }
        n++;
    }
    prev_header_ = lidar_msg_.header;
    } 

    // Initialize the new map to be published
    full_map.header.frame_id = "/base_link";
    full_map.info.resolution = map_res_;
    full_map.info.width = map_width_;
    full_map.info.height = map_height_;
    full_map.info.origin.position.x = -map_width_*map_res_ / 2;//map_height_ * map_res_;
    full_map.info.origin.position.y = 0;//-map_width_ * map_res_ / 2;
    full_map.info.origin.orientation =
                tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    full_map.data.resize(map_height_*map_width_);

    for (int i = 0; i < map_height_*map_width_; i++)
    {
        full_map.data[i] = belief_map_[i];
    }

    //Joins the occupancy grid of the lane and the lidar
    CombineOccupancyGrid(full_map, lane_detection_msg_, offset_height_, offset_width_);

    map_pub_.publish(full_map);
    belief_map_.clear();
    belief_map_.resize(map_width_*map_height_);
}

void LaserMapper::InitializeSubscribersandPublishers() {
    // PUBLISHERS
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid_output", 1);

    // SUBSCRIBERS
    lidar_sub_ = nh_.subscribe("/scan", 1, &LaserMapper::LidarCallback, this);
    lane_detection_sub_ = nh_.subscribe("/point_vec_out", 1, &LaserMapper::LaneDetectionCallback, this);
}

/**
 * @name LidarCallback
 * @brief Obtains message sent by Lidar
 *        & Processes it to belief_map_
 * @param[in] msg: Lidar Message
 * @return NONE
 */
void LaserMapper::LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    /* 
    Lidar, Left Lane, Right Lane callbacks all require
    DeleteValues - Deletes values of the 
                    belief map based on how much it moved
    StitchMap - Attaches the new map onto the front of the map
    */
    lidar_msg_ = *msg;
}

void LaserMapper::LaneDetectionCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    lane_detection_msg_ = *msg;
}

void LaserMapper::GetParam() {
    // Map
    nh_.param<double>("LaserMapper/map_res", map_res_, 001);
    nh_.param<int>("LaserMapper/map_W", map_width_, 1280);
    nh_.param<int>("LaserMapper/map_H", map_height_, 720);
    
    // Obstacle
    nh_.param<double>("LaserMapper/LASER_ORIENTATION", LASER_ORIENTATION_, -1);
    nh_.param<int>("LaserMapper/INFLATE_OBS", inflate_obstacle_, 8);

    // Scan
    nh_.param<double>("LaserMapper/max_angle", max_angle_, 3.14/2.0);
    nh_.param<double>("LaserMapper/min_angle", min_angle_, -3.14/2.0);
    nh_.param<double>("LaserMapper/minrange", min_range_, 0.2);
    nh_.param<double>("LaserMapper/maxrange", max_range_, 8);
}

void LaserMapper::InitMap() {
    belief_map_.reserve(map_height_*map_width_);
}

/**
 * @name UpdateLaserMap
 * @brief Updates the belief map with 
 *        new occupany grid value if valid
 * @param[in] x: width comparator
 * @param[in] y: height comparator
 * @param[in] value: occupancy grid value
 * @return NONE
 */
void LaserMapper::UpdateLaserMap(const int& x, const int& y, const double& value) {
  if (abs(x) < map_width_/2 && y < map_height_ && y > 0) {
    int map_index = (map_width_/2 - x) + (y)*map_width_;
    belief_map_[map_index] = value;
  }
}

/**
 * @name CheckMap
 * @brief Checks value of the cell of occupancy grid 
 * @param[in] x: width comparator
 * @param[in] y: height comparator
 * @return double: value of occupany grid at belief map
 */
//TODO Change this function to have less redundant code (Duplicate with UpdateLaserMap)
double LaserMapper::CheckMap(const int& x, const int& y)
{
  if (abs(x) < map_width_/2 && y < map_height_ && y > 0)
  {
    int map_index = (map_width_/2 - x) + (map_height_ - y)*map_width_;
    return belief_map_[map_index];
  }
  return OBS_;
}

/**
 * @name DrawLine
 * @brief Checks value of the cell of occupancy grid 
 *        The algorithm used below is called the Bresenham's line algorithm
 *        Wiki Lik: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 * This function is used for defining 3D objects onto a 2D space on the occupancy grid
 * @param[in] angle: width comparator
 * @param[in] range: height comparator
 * @param[in] inflate_factor: magnification value
 * @return NONE
 */
void LaserMapper::DrawLine(const float& angle, const float& range, const int& inflate_factor)
{
  // Checks if the range is reasonable
  if (range < min_range_ || range > max_range_)
    return;
  
  int x1 = LASER_ORIENTATION_*round(sin(angle)*range/map_res_);
  int y1 = round(cos(angle)*range/map_res_);

  UpdateLaserMap(x1, y1, OBS_);

  int dx = abs(x1);
  int dy = abs(y1);
  int sx, sy;

  if (x1 > 0)
    sx = 1;
  else
    sx = -1;

  if (y1 > 0)
    sy = 1;
  else
    sy = -1;

  int err = dx-dy;
  int x = 0;
  int y = 0;

  while (true) {
    if (CheckMap(x, y) < OBS_)
      UpdateLaserMap(x, y, NO_OBS_);

    if (x == x1 && y == y1) {
      for (int i = -inflate_factor; i < inflate_factor; i++) {
        for (int j = -inflate_factor; j < inflate_factor; j++) {
          UpdateLaserMap(x+i, y+j, OBS_);
        }
      }
      return;
    }

    int e2 = 2*err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }

    if (x == x1 && y == y1) {
      for (int i = -inflate_factor; i < inflate_factor; i++)
        for (int j = -inflate_factor; j < inflate_factor; j++)
          UpdateLaserMap(x+i, y+j, OBS_);
        return;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  } 
}

/** 
 *  @name CombineOccupancyGrid
 *  @brief joins two occupancy grids with the same resolution
 * 
 *  Joints 2 occupancy grid given a set of grid. The from_grid will be resized based on 
 *  the size of the old grid and the values will be added to the occupancy grid. In our
 *  case we use binary, so no need to actually add but do a bitwise or. Once the grid has 
 *  been joined, the pointer to the occupancy grid is returned.
 *  
 *  @param to_grid a grid to be joined
 *  @param from_grid destination grid to be joined
 *  @param offsetHeight an offset factored into the join
 *  @param offsetWidth an offset factored into the join
 *  @param start_val the starting value to slot into
 *         (Used for storing previous values)
 *  @return NONE
*/
void LaserMapper::CombineOccupancyGrid(nav_msgs::OccupancyGrid &to_grid, 
                                      const nav_msgs::OccupancyGrid &from_grid, 
                                      const int offset_height, 
                                      const int offset_width) 
{
    // Load The two grids
  if (&to_grid == NULL || &from_grid == NULL)
  {
    ROS_ERROR("CombineOccupancyGrid: Grid object NULL.");
    return;
  }

  int resolution = to_grid.info.resolution;

  // TODO(jungwook): Handle Resolution Change
  if (to_grid.info.resolution != from_grid.info.resolution)
  {
    ROS_ERROR("CombineOccupancyGrid: Resolution mismatch.");
    return;
  }

  // Copy content to the to the original grid
  for (int j = 0; j < from_grid.info.height; j++)
  {
    for (int i = 0; i < from_grid.info.width; i++)
    {
      // Check for valid size before modifications
      if (i+offset_width < to_grid.info.width && j+offset_height < to_grid.info.height)
      {
        // Do operation
        int index = ijToIndex(i+offset_width, j+offset_height, to_grid.info.width);
        int from_index = ijToIndex(i, j, from_grid.info.width);
        if (from_grid.data[from_index] > 0) // OBS are larger than 0
          to_grid.data[index] = from_grid.data[from_index];
      }
    }
  }
  return;
}

/** 
 * @name ijToIndex
 * @brief Returns index of given (i,j) from a grid given max_width and max_height
 * 
 *  Parameters: starting location of the grid and the height and width of the new grid.
 *  Starts from (0,0)
 *  --------> i
 *  ********* |
 *  ********* |
 *  ********* |
 *  ********* v j 
 *
 *  @parma i starting location of the grid(x-axis)
 *  @parma j starting location of the grid(y-axis)
 *  @parma max_width the width of the new grid.
*/
int LaserMapper::ijToIndex(const int i, const int j, const int max_width ) { return floor(((max_width*j)-1)+i); }

/**
 * @name ShiftMap
 * @brief Shifts the map based on x y movement
 * @param[in] prev_map: map that needs to be updated
 * @return Updates the prev_map with shifting map
 */
std::vector<int> LaserMapper::ShiftMap(std::vector<int> prev_map) {
  std::vector<int> shift_map(map_width_*map_height_);
  if(prev_map.empty()){
    ROS_ERROR("There is no map to shift!");
    return shift_map;
  } 
  else {
    shift_map = prev_map;
  }
  
  tf::StampedTransform transform;
  try {
    position_listener_.lookupTransform("/odom", "/base_link",
                              ros::Time(0), transform);

  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return shift_map;
  }

  // Assumes previous value is given
  if (prev_x_ != 0 && prev_y_ != 0) {
    /*Gains the difference in x and y transition
      If the diff value is negative it is 
      either fill left or bottom (vice versa)
    */
    int diff_x = transform.getOrigin().x() - prev_x_;
    int diff_y = transform.getOrigin().y() - prev_y_;

    //Filling UNKNOWN for x
    if (diff_x > 0) {
      for(int i = 0; i < map_height_; i++) {
        std::vector<int> temp(map_width_);
        std::copy(shift_map.begin(), shift_map.begin()+map_width_, temp.begin());
        std::rotate(temp.begin(), temp.begin()+abs(diff_x), temp.end());
        std::fill(temp.rbegin(), temp.rbegin()+abs(diff_x), UNKNOWN_);
        std::copy(temp.begin(), temp.end(), shift_map.begin());
      }

      // Replaces functionality with std::fill()
      // for(int i = 1; i <= map_height_; i++) {
      //   for(int j = 1; j <= abs(diff_x); j++) {
      //     shift_map[(j*i)-1] = UNKNOWN_;
      //   }
      // }

    }
    else {
      for(int i = 0; i < map_height_; i++) {
        std::vector<int> temp(map_width_);
        std::copy(shift_map.begin(), shift_map.begin()+map_width_, temp.begin());
        std::rotate(temp.begin(), temp.begin()+abs(diff_x), temp.end());
        std::fill(temp.begin(), temp.begin()+abs(diff_x), UNKNOWN_);
        std::copy(temp.begin(), temp.end(), shift_map.begin());
      }

      // Replaces functionality with std::fill()
      // for(int i = map_height_; i >= 1; i--) {
      //   for(int j = abs(diff_x); j >= 1; j--) {
      //     shift_map[(j*i)-1] = UNKNOWN_;
      //   }
      // }
    }
    
    //Filling UNKNOWN for y
    if (diff_y > 0) {
      //'Rotates' the map dragging all values 
      //By abs(diff_y)*map_width_ amount
      std::rotate(shift_map.begin(), 
          shift_map.begin()+(abs(diff_y)*map_width_), 
          shift_map.end());

      for(int i = 0; i < abs(diff_y)*map_width_; i++) {
        shift_map[i] = UNKNOWN_;
      }
    }
    else {
      std::rotate(shift_map.rbegin(), 
          shift_map.rbegin()+(abs(diff_y)*map_width_), 
          shift_map.rend());

      for(int i = (abs(diff_y)*map_width_)-1; i >= 0; i--) {
        shift_map[i] = UNKNOWN_;
      }
    }
  }

  prev_x_ = transform.getOrigin().x();
  prev_y_ = transform.getOrigin().y();

  //Rotation
  double diff_ang = transform.getRotation().getAngle() - prev_ang_;
  shift_map = RotateMap(shift_map, diff_ang);
  prev_ang_ = transform.getRotation().getAngle();
  return shift_map;
}

/**
 * @name RotateMap
 * @brief Rotates the map based on angular rotation
 * @param[in] curr_map: current map to analyze
 * @param[in] new_ang: angular difference from previous location
 * @return rot_map: newly rotated map
 */
std::vector<int> LaserMapper::RotateMap(std::vector<int> curr_map, double new_ang){
  std::vector<LaserMapper::CellEntity> cell_map;
  std::vector<int> rot_map;
  //Return a empty Cell Map if no value is input
  if(curr_map.empty()){
    ROS_ERROR("LaserMapper::RotateMap: Map is empty!");
    return rot_map;
  }

  //Generate CellEntity Vector & Rotate
  for(int i = 0; i < map_height_; i++){
    for(int j = 1; j <= map_width_; j++){
      int curr_x = j - map_width_/2;
      int curr_y = map_height_ - i;
      LaserMapper::CellEntity curr_en;
      curr_en.val = curr_map.at(i*map_height_ + j - 1);
      curr_en.length = sqrt(curr_x*curr_x +
                        curr_y*curr_y);
      curr_en.angle = atan2(curr_y, curr_y) + new_ang;
      curr_en.xloc = rint(curr_en.length*cos(curr_en.angle));
      curr_en.yloc = rint(curr_en.length*sin(curr_en.angle));
      cell_map.push_back(curr_en);
    }
  }

  //Generates a map of unknwons same size as the belief map
  rot_map.resize(map_width_*map_height_, UNKNOWN_);

  //Fills in values based on criteria  
  for(int i = 0; i < rot_map.size(); i++){
    LaserMapper::CellEntity curr_en = cell_map.at(i);
    //Checks for bounds of x & y
    if((curr_en.xloc >= 0 && curr_en.xloc <= map_width_) && 
        (curr_en.yloc >= 0 && curr_en.yloc <= map_height_)){
      rot_map[i] = curr_en.val;
    }
  }

  return rot_map;
}