#include "rrt_star_rs/rrt_star_rs_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


RRTStarRSFlow::RRTStarRSFlow(ros::NodeHandle &nh) {
    constants_.front_hang_length = nh.param("planner/front_hang_length",0.96);
    constants_.rear_hang_length = nh.param("planner/rear_hang_length",0.929);
    constants_.wheel_base = nh.param("planner/wheel_base", 2.8);
    constants_.vehicle_length = constants_.front_hang_length + constants_.rear_hang_length + constants_.wheel_base;
    constants_.vehicle_width = nh.param("planner/vehicle_width",1.942);
    constants_.vel_max = nh.param("planner/velocity_max",2.5); // m/s 
    constants_.vel_min = nh.param("planner/velocity_min",2.5);// all positive number;
    constants_.accel_limit = nh.param("planner/acceleration_limit",1); // m/s^2
    constants_.decel_limit = nh.param("planner/deceleration_limit",1); 
    constants_.steering_angle = nh.param("planner/steering_angle", 0.75); // rad
    constants_.ang_vel_max = nh.param("planner/angular_velocity_max",0.5);// rad/s
    constants_.radius_thres = nh.param("planner/radius_thres",10);
    constants_.safety_distance = nh.param("planner/safety_distance",0.1);
    constants_.move_step_size = nh.param("planner/move_step_size",0.5);
    constants_.max_iter = nh.param("planner/max_iter",200);
    constants_.connect_circle_dist = nh.param("planner/connect_circle_dist",50);
    constants_.expand_dist = nh.param("planner/expand_dist",30);
    constants_.goal_dist_margin = nh.param("planner/goal_dist_margin",0.5);
    constants_.goal_yaw_margin = nh.param("planner/goal_yaw_margin",0.3183);


    // map config infomation
    constants_.map_grid_resolution = nh.param("planner/map_grid_resolution",0.05);
    constants_.csv_path = nh.param<std::string>("csv_path", "/home/linkx/lkx_ws/src/tpcap/TPCAP_benchmarks/Case1.csv");
    constants_.case_order = nh.param("case_order",22);
    constants_.radius_min = constants_.wheel_base / tan(constants_.steering_angle);


    kinodynamic_rrt_star_ptr_ = std::make_shared<RRTStar>(constants_);
    Trajectory_ptr = std::make_shared<Trajectory>(constants_);

    path_pub_ = n.advertise<nav_msgs::Path>("/searched_path", 1);
    searched_tree_pub_ = n.advertise<visualization_msgs::Marker>("/searched_tree", 1);
    point_arrow_pub_ = n.advertise<visualization_msgs::MarkerArray>("/point_arrow",1);
    vehicle_path_pub_ = n.advertise<visualization_msgs::MarkerArray>("/vehicle_path", 1);
    pose_pub_ = n.advertise<visualization_msgs::MarkerArray>("/pose_array",1);
}

void RRTStarRSFlow::Run() {
     
    kinodynamic_rrt_star_ptr_->Init(constants_);
    int best_index = kinodynamic_rrt_star_ptr_->Search();

    if ( best_index != -1) {
        
        path_ = kinodynamic_rrt_star_ptr_->getPath(best_index);
        Trajectory_ptr->SetPoints(path_);
        Trajectory_ptr->SetVelocity();
        Trajectory_ptr->Plot();
        Trajectory_ptr->PlotVelocity();
    }
    kinodynamic_rrt_star_ptr_->GetRRTtree();
    kinodynamic_rrt_star_ptr_->GetAllPoint();
}



void RRTStarRSFlow::PublishRviz()
{
    PublishStartAndGoalPose(kinodynamic_rrt_star_ptr_->start_state_,
                        kinodynamic_rrt_star_ptr_->goal_state_);
    PublishPath(path_);
    PublishVehiclePath(path_, kinodynamic_rrt_star_ptr_->vehicle_length_, 
                        kinodynamic_rrt_star_ptr_->vehicle_width_,5u);

    PublishSearchedTree(kinodynamic_rrt_star_ptr_->line_tree);
    PublishPointArrow(kinodynamic_rrt_star_ptr_->candidate_points);
}



void RRTStarRSFlow::PublishPath(const VectorVec4d &path) {
    nav_msgs::Path nav_path;

    geometry_msgs::PoseStamped pose_stamped;
    ros::Time time_ = ros::Time::now();

    for (const auto &pose: path) {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose[0];
        pose_stamped.pose.position.y = pose[1];
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose[2]);

        nav_path.poses.emplace_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = timestamp_;

    path_pub_.publish(nav_path);
}

void RRTStarRSFlow::PublishVehiclePath(const VectorVec4d &path, double length,
                                         double width, unsigned int vehicle_interval = 5u) {
    visualization_msgs::MarkerArray vehicle_array;

    for (unsigned int i = 0; i < path.size(); i += vehicle_interval) {
        visualization_msgs::Marker vehicle;

        if (i == 0) {
            vehicle.action = 3;
        }

        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = static_cast<int>(i / vehicle_interval);
        vehicle.scale.x = length;
        vehicle.scale.y = width;
        vehicle.scale.z = 0.01;

        vehicle.color.a = 0.1;
        vehicle.color.r = 1.0;
        vehicle.color.b = 0.0;
        vehicle.color.g = 0.0;


        vehicle.pose.position.x = path.at(i)[0];
        vehicle.pose.position.y = path.at(i)[1];
        vehicle.pose.position.z = 0.0;

        vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(path.at(i)[2]);
        vehicle_array.markers.emplace_back(vehicle);
    }

    vehicle_path_pub_.publish(vehicle_array);
}

void RRTStarRSFlow::PublishSearchedTree(const VectorVec4d &searched_tree) {
    visualization_msgs::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = ros::Time::now();
    tree_list.type = visualization_msgs::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 1.0;
    tree_list.color.r = 0;
    tree_list.color.g = 0;
    tree_list.color.b = 0;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::Point point;
    for (const auto &i: searched_tree) {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.emplace_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.emplace_back(point);
    }
    searched_tree_pub_.publish(tree_list);
}

void RRTStarRSFlow::PublishPointArrow(const VectorVec3d &pt_lists){
    visualization_msgs::MarkerArray arrow_array;
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = "world";
    arrow_marker.header.stamp = ros::Time();
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.scale.x = 0.6;
    arrow_marker.scale.y = 0.2;
    arrow_marker.scale.z = 0.1;
    arrow_marker.color.r = 1.0;
    arrow_marker.color.a = 1.0;
    
    int count  = 1;
    for(const auto &pt:pt_lists){
        arrow_marker.id = count;
        arrow_marker.pose.position.x = pt[0];
        arrow_marker.pose.position.y = pt[1];
        arrow_marker.pose.position.z = 0.0;
        arrow_marker.pose.orientation = tf::createQuaternionMsgFromYaw(pt[2]);
        arrow_array.markers.push_back(arrow_marker);
        count++;
    }

    point_arrow_pub_.publish(arrow_array);

}



void RRTStarRSFlow::PublishStartAndGoalPose(const Vec3d &start_pose, const Vec3d &goal_pose)
{
    visualization_msgs::MarkerArray pose_array;

    // start_pose
    visualization_msgs::Marker start;
    start.header.frame_id = "world";
    start.header.stamp = ros::Time::now();
    start.id = 0;
    start.type = visualization_msgs::Marker::ARROW;
    start.action = visualization_msgs::Marker::ADD;
    start.lifetime = ros::Duration();

    start.color.r = 0.0f;
    start.color.b = 0.0f;
    start.color.g = 1.0f;
    start.color.a = 1.0;

    start.scale.x = 2;
    start.scale.y = 0.2;
    start.scale.z = 0.01;

    start.pose.position.x = start_pose[0];
    start.pose.position.y = start_pose[1];
    start.pose.position.z = 0.0;
    start.pose.orientation = tf::createQuaternionMsgFromYaw(start_pose[2]);
    pose_array.markers.emplace_back(start);

    // goal pose
    visualization_msgs::Marker goal;
    goal.header.frame_id = "world";
    goal.header.stamp = ros::Time::now();
    goal.id = 1;
    goal.type = visualization_msgs::Marker::ARROW;
    goal.action = visualization_msgs::Marker::ADD;
    goal.lifetime = ros::Duration();

    goal.color.r = 1.0f;
    goal.color.b = 0.0f;
    goal.color.g = 0.0f;
    goal.color.a = 1;

    goal.scale.x = 2;
    goal.scale.y = 0.2;
    goal.scale.z = 0.01;

    goal.pose.position.x = goal_pose[0];
    goal.pose.position.y = goal_pose[1];
    goal.pose.position.z = 0.0;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose[2]);
    pose_array.markers.emplace_back(goal);

    for(int i = 0;i<2;++i)
    {
        visualization_msgs::Marker vehicle;
        vehicle.header.frame_id = "world";
        vehicle.header.stamp = ros::Time::now();
        vehicle.id = i+2;
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.action = visualization_msgs::Marker::ADD;
        vehicle.lifetime = ros::Duration();

        vehicle.color.r = 0.0f;
        vehicle.color.b = 0.5f;
        vehicle.color.g = 0.5f;
        vehicle.color.a = 0.3;

        vehicle.scale.x = 4.689;
        vehicle.scale.y = 1.942;
        vehicle.scale.z = 0.01;
 

        if(i == 0)
        {
            vehicle.pose.position.x = start_pose[0] + 1.4155*cos(start_pose[2]);
            vehicle.pose.position.y = start_pose[1] + 1.4155*sin(start_pose[2]);
            vehicle.pose.position.z = 0.0;
            vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(start_pose[2]);
        }

        else if(i == 1){
            vehicle.pose.position.x = goal_pose[0] + 1.4155*cos(goal_pose[2]);
            vehicle.pose.position.y = goal_pose[1] + 1.4155*sin(goal_pose[2]);
            vehicle.pose.position.z = 0.0;
            vehicle.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose[2]);

        }

        pose_array.markers.emplace_back(vehicle);
        
    }

    pose_pub_.publish(pose_array);

}
