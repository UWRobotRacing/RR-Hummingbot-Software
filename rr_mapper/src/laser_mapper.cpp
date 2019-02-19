/** @file laser_mapper.cpp
 *  @author Jungwook Lee
 *  @author Andrew Jin (DongJunJin)
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */

// CPP
#include <algorithm>

// Local
#include "laser_mapper.hpp"
#include <occupancy_grid_utils.hpp>

/**
 * @name LaserMapper
 * @brief initiliazes the LaserMapper class
 * @return NONE
 */
LaserMapper::LaserMapper(ros::NodeHandle nh) : it_(nh) {
}

/**
 * @name ~LaserMapper
 * @brief destructs the LaserMapper class
 * @return NONE
 */
LaserMapper::~LaserMapper() {
}
