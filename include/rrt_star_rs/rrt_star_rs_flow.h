#ifndef _RRT_STAR_RS_FLOW_
#define _RRT_STAR_RS_FLOW_

#include "rrt_star.h"
#include "type.h"
#include "timer.h"
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
    double calculateDist(const VectorVec4d &path);

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
