# ifndef __CONSTANTS_H__
# define __CONSTANTS_H__

#include <iostream>
#include <string>
#include "rrt_star_rs/type.h"

class constants
{
public:
    double front_hang_length = 0.96;
    double rear_hang_length = 0.929;
    double wheel_base = 2.8;
    double vehicle_length;
    double vehicle_width = 1.942;
    double vel_max = 2.5; // m/s 
    double vel_min = 2.5; // all positive number;
    double accel_limit = 1; // m/s^2
    double decel_limit = 1; 
    double steering_angle = 0.75; // rad
    double ang_vel_max = 0.5;// rad/s
    double radius_thres = 10;
    double safety_distance = 0.1;


    // map config infomation
    double map_grid_resolution = 0.05;

    // minimum steering radius
    double radius_min = 0;
   
    // case number
    std::string csv_path = "/home/linkx/lkx_ws/src/tpcap/tpcap_scene/case1.csv";
    int case_order = 22;
    
    Vec3d start_state_original;
    Vec3d goal_state_original;
    
    int map_grid_size_x;
    int map_grid_size_y;

    double map_x_lower;
    double map_x_upper;
    double map_y_lower;
    double map_y_upper;
    Vec2d origin;
    std::vector<VectorVec2d> obs_array;
    double minRoadWidth = 2;

    // for RRT algorithm
    double move_step_size;
    int max_iter;
    double connect_circle_dist;
    double expand_dist;
    double goal_dist_margin;
    double goal_yaw_margin;


  constants(/* args */){};
  ~constants(){};
};


# endif
