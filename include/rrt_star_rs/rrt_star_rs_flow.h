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

#ifndef _RRT_STAR_RS_FLOW_
#define _RRT_STAR_RS_FLOW_

#include "rrt_star.h"
#include "type.h"
#include <ros/ros.h>
#include "constants.h"
#include "velocity_set.h"
#include <iostream>

class RRTStarRSFlow {
public:

    RRTStarRSFlow() = default;

    explicit RRTStarRSFlow(ros::NodeHandle &nh);

    void Run();
    void PublishRviz();

    std::shared_ptr<RRTStar> kinodynamic_rrt_star_ptr_;
    std::shared_ptr<Trajectory> Trajectory_ptr;
    constants constants_;

private:
    void PublishPath(const VectorVec4d &path);
    void PublishVehiclePath(const VectorVec4d &path, double width,
                            double length, unsigned int vehicle_interval);
    void PublishStartAndGoalPose(const Vec3d &start_pose, const Vec3d &goal_pose);

    void PublishSearchedTree(const VectorVec4d &searched_tree);
    void PublishPointArrow(const VectorVec3d &pt_lists);

private:
    ros::NodeHandle n;
    ros::Publisher path_pub_;
    ros::Publisher searched_tree_pub_;
    ros::Publisher point_arrow_pub_;
    ros::Publisher vehicle_path_pub_;
    ros::Publisher pose_pub_;

    VectorVec4d path_;

    ros::Time timestamp_;
};

#endif //HYBRID_A_STAR_HYBRID_A_STAR_FLOW_H
