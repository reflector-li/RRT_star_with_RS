#ifndef _RRT_STAR_H_
#define _RRT_STAR_H_

#include <ros/ros.h>
#include <iostream>
#include "constants.h"
#include "rs_path.h"
#include "state_node.h"
#include "file_tools.h"

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc.hpp>

#include <glog/logging.h>

#include <map>
#include <memory>
#include <stdlib.h>
#include <time.h>

class RRTStar {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3d start_state_, goal_state_;
    double vehicle_length_, vehicle_width_;
    constants params_;
    std::shared_ptr<cv::Mat> img_ptr_;
    VectorVec4d line_tree;
    VectorVec3d candidate_points;

    RRTStar() = delete;

    RRTStar(const constants &params);

    ~RRTStar();

    void Init(constants &params);
    int Search();

    void GetRRTtree();
    void GetAllPoint();

    VectorVec4d getPath(int best_index) const;

    void SetVehicleShape(double front_hang_length, double rear_hang_length, double wheel_base, double vehicle_width);

    void Reset();

private:
    inline bool HasObstacle(int grid_index_x, int grid_index_y) const;
    inline bool HasObstacle(const Vec2i &grid_index) const;
    bool CheckCollision(const double &x, const double &y, const double &theta);
    __attribute__ ((optimize(0))) inline bool LineCheck(double x0, double y0, double x1, double y1);


    inline Vec2d MapGridIndex2Coordinate(const Vec2i &grid_index) const;
    Vec2i Coordinate2MapGridIndex(const Vec2d &pt) const;

    void GetNeighborNodes(const StateNode::Ptr &curr_node_ptr, std::vector<StateNode::Ptr> &neighbor_nodes,double step_size);

    inline void DynamicModel(const double &step_size, const double &phi, double &x, double &y, double &theta) const;
    bool BeyondBoundary(const Vec2d &pt) const;
    static inline double Mod2Pi(const double &x);
    void ReleaseMemory();

  // for rrt *
    Vec3d getRandomPos();
    int getNearestIndex(const Vec3d &pose);
    StateNode::Ptr steer(const StateNode::Ptr &nearest_node,Vec3d rand_pose);
    bool checkPathCollision(const VectorVec3d &path);
    std::vector<int> findNearNodes(const StateNode::Ptr &node);
    void resetParent(StateNode::Ptr &node, const std::vector<int> &indexs);
    void rewire(const StateNode::Ptr &node, const std::vector<int> &indexs);
    void tryGoalPath(const StateNode::Ptr &node);
    int searchBestGoalNode();





private:
    Vec2d origin_; 

    StateNode::Ptr root_node_ptr_ = nullptr;
    std::vector<StateNode::Ptr> RRTtree;


    double path_length_ = 0.0;

    std::shared_ptr<RSPath> rs_path_ptr_;

    VecXd vehicle_shape_;
    MatXd vehicle_shape_discrete_;

    // debug
    double check_collision_use_time = 0.0;
    int num_check_collision = 0;
    int visited_node_number_ = 0;
};

#endif 
