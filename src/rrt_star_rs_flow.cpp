/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "rrt_star_rs/rrt_star_rs_flow.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


RRTStarRSFlow::RRTStarRSFlow(ros::NodeHandle &nh) {

   
}

void RRTStarRSFlow::Run() {
     
     // 创建栅格地图，起点位姿和终点位姿，其中使用cv::Mat 作为栅格地图
     kinodynamic_rrt_star_ptr_->Init(constants_);

    if (kinodynamic_rrt_star_ptr_->Search() != -1) {
        
        path_ = kinodynamic_rrt_star_ptr_->getPath();
        
    }
}



void RRTStarRSFlow::PublishRviz()
{
    PublishStartAndGoalPose(kinodynamic_rrt_star_ptr_->start_state_,
                        kinodynamic_rrt_star_ptr_->goal_state_);
    PublishPath(path_);
    PublishVehiclePath(path_, kinodynamic_rrt_star_ptr_->vehicle_length_, 
                        kinodynamic_rrt_star_ptr_->vehicle_width_,5u);
    PublishSearchedTree(kinodynamic_rrt_star_ptr_->GetRRTtree());
    
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
