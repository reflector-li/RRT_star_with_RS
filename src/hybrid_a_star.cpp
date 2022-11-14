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

#include "rrt_star_rs/rrt_star.h"
#include "rrt_star_rs/display_tools.h"
#include "rrt_star_rs/timer.h"

#include <iostream>

HybridAStar::HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                         int segment_length_discrete_num, double steering_penalty,
                         double reversing_penalty, double steering_change_penalty, double shot_distance,
                         double front_hang_length, double rear_hang_length, double wheel_base, double vehicle_width,
                         double map_grid_resolution, double state_grid_resolution, std::string csv_path, int grid_size_phi) {
    wheel_base_ = wheel_base;
    segment_length_ = segment_length;
    steering_radian_ = steering_angle; // use radius
    steering_discrete_num_ = steering_angle_discrete_num;
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;
    move_step_size_ = segment_length / segment_length_discrete_num;
    reverse_move_step_size_ = move_step_size_/5;
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);
    steering_penalty_ = steering_penalty;
    steering_change_penalty_ = steering_change_penalty;
    reversing_penalty_ = reversing_penalty;
    shot_distance_ = shot_distance;

    MAP_GRID_RESOLUTION_ = map_grid_resolution;
    STATE_GRID_RESOLUTION_ = state_grid_resolution;
    CSV_PATH_ = csv_path;

    vehicle_length_ = front_hang_length + rear_hang_length + wheel_base;
    vehicle_width_ = vehicle_width;

    SetVehicleShape(front_hang_length,rear_hang_length, wheel_base, vehicle_width);

    CHECK_EQ(static_cast<float>(segment_length_discrete_num_ * move_step_size_), static_cast<float>(segment_length_))
        << "The segment length must be divisible by the step size. segment_length: "
        << segment_length_ << " | step_size: " << move_step_size_;

    //根据最大转向角获得最小转弯半径
    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));
    tie_breaker_ = 1.0 + 1e-3;

    STATE_GRID_SIZE_PHI_ = grid_size_phi;
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0 ;
}

HybridAStar::~HybridAStar() {
    ReleaseMemory();
}

double HybridAStar::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

void HybridAStar::Init()
{
    std::vector<double> map_raw_data;
    ReadCsv(CSV_PATH_, map_raw_data);
    std::cout<<CSV_PATH_<<std::endl;

    //set start point and goal point
    start_state_ = {map_raw_data[0],map_raw_data[1],Mod2Pi(map_raw_data[2])};
    goal_state_ = {map_raw_data[3],map_raw_data[4],Mod2Pi(map_raw_data[5])};

    // for reverseSearch
    target_yaw_ = Mod2Pi((start_state_[2]+goal_state_[2])/2);
    weight_length_ = 1.0;
    weight_yaw_ = 40.0;


    int obs_num = static_cast<int>(map_raw_data[6]);
    ROS_INFO("scene has %d obstacles",obs_num);

    int offset = 6+obs_num;

    std::vector<std::vector<cv::Point2d>> obs_array_cardesian;
    std::vector<std::vector<cv::Point2i>> obs_array_grid;

    double low_x = start_state_[0] < goal_state_[0]? start_state_[0]:goal_state_[0];
    double upper_x = start_state_[0] > goal_state_[0]? start_state_[0]:goal_state_[0];
    double low_y = start_state_[1] < goal_state_[1]? start_state_[1]:goal_state_[1];
    double upper_y = start_state_[1] > goal_state_[1]? start_state_[1]:goal_state_[1];

    // low_x -= vehicle_length_;
    // upper_x += vehicle_length_;
    // low_y -= vehicle_length_;
    // upper_y += vehicle_length_; 

    // double low_x = map_raw_data[offset+1], low_y=map_raw_data[offset+2];
    // double upper_x = low_x, upper_y = low_y;
    for(int i=1;i<=obs_num;i++)
    {
        std::vector<cv::Point2d> points_array_cardesian;
        int point_num = map_raw_data[6+i];
        for(int j=1;j<=point_num;++j)
        {
            cv::Point2d point;
            if(map_raw_data[offset+2*j-1]>upper_x)
                upper_x = map_raw_data[offset+2*j-1];
            else if(map_raw_data[offset+2*j-1]<low_x)
                low_x = map_raw_data[offset+2*j-1];
            if(map_raw_data[offset+2*j]>upper_y)
                upper_y = map_raw_data[offset+2*j];
            else if(map_raw_data[offset+2*j]<low_y)
                low_y = map_raw_data[offset+2*j];

            point.x = map_raw_data[offset+2*j-1];
            point.y = map_raw_data[offset+2*j];
            points_array_cardesian.push_back(point);

        }
        obs_array_cardesian.push_back(points_array_cardesian);
        offset += 2*point_num;
    }

    double margin = 5 ;

    map_x_lower_ = low_x - margin;
    map_x_upper_ = upper_x + margin;
    map_y_lower_ = low_y - margin;
    map_y_upper_ = upper_y + margin;
    
    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);

    MAP_GRID_SIZE_X_ = floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    MAP_GRID_SIZE_Y_ = floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

    state_node_map_ = new StateNode::Ptr **[STATE_GRID_SIZE_X_];

    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        state_node_map_[i] = new StateNode::Ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            state_node_map_[i][j] = new StateNode::Ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }

    map_data_ = cv::Mat(MAP_GRID_SIZE_Y_,MAP_GRID_SIZE_X_,CV_8UC1,cv::Scalar(0));
    origin_ = {map_x_lower_,map_y_lower_};
    for(auto &obs:obs_array_cardesian)
    {
        std::vector<cv::Point2i> points_array_grid;
        for(auto &point:obs)
        {
            cv::Point2i point_grid;

            point_grid.x = static_cast<int>((point.x - origin_.x) / MAP_GRID_RESOLUTION_);
            point_grid.y = static_cast<int>((point.y - origin_.y) / MAP_GRID_RESOLUTION_);
            points_array_grid.push_back(point_grid);
        }
        obs_array_grid.push_back(points_array_grid);
    }
    // cv::Point2i pt1(58,175),pt2(36,182);
    cv::drawContours(map_data_, obs_array_grid, -1, cv::Scalar(255), -1, 8);
    // cv::circle(map_data_,pt1,2,cv::Scalar(125),-1);
    // cv::circle(map_data_,pt2,2,cv::Scalar(125),-1);
    // std::cout<<"mat image:"<<static_cast<int>(map_data_.at<uchar>(175,58))<<" "<<static_cast<int>(map_data_.at<uchar>(182,36))<<std::endl;
    // std::cout<<(map_data_.at<uchar>(175,58) == 125)<<std::endl;
    // cv::imshow("grid_map",map_data_);
    // cv::waitKey(0);
}


inline bool HybridAStar::LineCheck(double x0, double y0, double x1, double y1) {
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

    if (steep) {
        std::swap(x0, y0);
        std::swap(y1, x1);
    }

    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    auto delta_x = x1 - x0;
    auto delta_y = std::abs(y1 - y0);
    auto delta_error = delta_y / delta_x;
    decltype(delta_x) error = 0;
    decltype(delta_x) y_step;
    auto yk = y0;

    if (y0 < y1) {
        y_step = 1;
    } else {
        y_step = -1;
    }

    auto N = static_cast<unsigned int>(x1 - x0);
    for (unsigned int i = 0; i < N; ++i) {
        if (steep) {
            // bool t1 = HasObstacle(Vec2i(yk, x0 + i * 1.0));
            // bool t2 = BeyondBoundary(Vec2d(yk * MAP_GRID_RESOLUTION_,
            //                             (x0 + i) * MAP_GRID_RESOLUTION_));
            if (HasObstacle(Vec2i(yk, x0 + i * 1.0))
                || BeyondBoundary(Vec2d(yk * MAP_GRID_RESOLUTION_+origin_.x,
                                        (x0 + i) * MAP_GRID_RESOLUTION_ + origin_.y))
                    ) {
                return false;
            }
        } else {
            // bool t1 = HasObstacle(Vec2i(x0 + i * 1.0, yk));
            // bool t2 = BeyondBoundary(Vec2d((x0 + i) * MAP_GRID_RESOLUTION_,
            //                             yk * MAP_GRID_RESOLUTION_));
            if (HasObstacle(Vec2i(x0 + i * 1.0, yk))
                || BeyondBoundary(Vec2d((x0 + i) * MAP_GRID_RESOLUTION_ + origin_.x,
                                        yk * MAP_GRID_RESOLUTION_ + origin_.y))) {
                return false;
            }
        }

        error += delta_error;
        if (error >= 1) {
            yk += y_step;
            error = error - 1.0;
        }
    }

    return true;
}

bool HybridAStar::CheckCollision(const double &x, const double &y, const double &theta) {
    Timer timer;
    Mat2d R;
    R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);

    VecXd transformed_vehicle_shape;
    transformed_vehicle_shape.resize(8);
    for (unsigned int i = 0; i < 4u; ++i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0)
                = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
    }

    Vec2i transformed_pt_index_0 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(0, 0)
    );
    Vec2i transformed_pt_index_1 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(2, 0)
    );

    Vec2i transformed_pt_index_2 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(4, 0)
    );

    Vec2i transformed_pt_index_3 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(6, 0)
    );

    double y1, y0, x1, x0;
    // pt1 -> pt0
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt2 -> pt1
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt3 -> pt2
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt0 -> pt3
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x0, y0, x1, y1)) {
        return false;
    }

    check_collision_use_time += timer.End();
    num_check_collision++;
    return true;
}

bool HybridAStar::HasObstacle(const int grid_index_x, const int grid_index_y) const {
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_.at<uchar>(grid_index_y,grid_index_x)==255));
}

bool HybridAStar::HasObstacle(const Vec2i &grid_index) const {
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_.at<uchar>(grid_index_y,grid_index_x)==255));
}


void HybridAStar::SetVehicleShape(double front_hang_length, double rear_hang_length, double wheel_base, double vehicle_width)
{
    
    /*   车辆坐标系以后轴中点为原点，以朝向为x轴，左边方向为y轴
         ^ y轴
      1  |            2
      ----------------------
         |            |
           ----------------->x轴
         |            |
      ---------------------
      4                3
   */ 
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_hang_length, vehicle_width/2); //车辆坐标系左后轮坐标向量
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(wheel_base + front_hang_length, vehicle_width / 2); //左前轮坐标
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(wheel_base + front_hang_length, -vehicle_width / 2);//右前轮坐标
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_hang_length, -vehicle_width / 2);//右后轮坐标

    const double step_size = move_step_size_;
    const auto N_length = static_cast<unsigned int>((front_hang_length + wheel_base + rear_hang_length) / step_size);
    const auto N_width = static_cast<unsigned int> (vehicle_width / step_size);
    //为车俩向前向后向左向右移动一个车位提供支持
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

    const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                     - vehicle_shape_.block<2, 1>(0, 0)).normalized();
    for (unsigned int i = 0; i < N_length; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i)
                = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                     - vehicle_shape_.block<2, 1>(2, 0)).normalized();
    for (unsigned int i = 0; i < N_width; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }

}


void HybridAStar::SetVehicleShape(double length, double width, double rear_axle_dist) {
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2); //车辆坐标系左后轮坐标向量
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2); //左前轮坐标
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);//右前轮坐标
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);//右后轮坐标

    const double step_size = move_step_size_;
    const auto N_length = static_cast<unsigned int>(length / step_size);
    const auto N_width = static_cast<unsigned int> (width / step_size);
    //为车俩向前向后向左向右移动一个车位提供支持
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

    const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                     - vehicle_shape_.block<2, 1>(0, 0)).normalized();
    for (unsigned int i = 0; i < N_length; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i)
                = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                     - vehicle_shape_.block<2, 1>(2, 0)).normalized();
    for (unsigned int i = 0; i < N_width; ++i) {
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}

__attribute__((unused)) Vec2d HybridAStar::CoordinateRounding(const Vec2d &pt) const {
    return MapGridIndex2Coordinate(Coordinate2MapGridIndex(pt));
}

Vec2d HybridAStar::MapGridIndex2Coordinate(const Vec2i &grid_index) const {
    Vec2d pt;
    pt.x() = ((double) grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + origin_.x;
    pt.y() = ((double) grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + origin_.y;

    return pt;
}

Vec3i HybridAStar::State2Index(const Vec3d &state) const {
    Vec3i index;

    index[0] = std::min(std::max(int((state[0] - origin_.x) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - origin_.y) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);
    // STATE_GRID_SIZE_PHI_ = 72
    return index;
}

Vec2i HybridAStar::Coordinate2MapGridIndex(const Vec2d &pt) const {
    Vec2i grid_index;

    grid_index[0] = int((pt[0] - origin_.x) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - origin_.y) / MAP_GRID_RESOLUTION_);
    return grid_index;
}

void HybridAStar::GetNeighborNodes(const StateNode::Ptr &curr_node_ptr,
                                   std::vector<StateNode::Ptr> &neighbor_nodes,double step_size) {
    neighbor_nodes.clear();
 
    // steering_discrete_num_ 默认是1
    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) {
        VectorVec3d intermediate_state;
        bool has_obstacle = false;

        double x = curr_node_ptr->state_.x();
        double y = curr_node_ptr->state_.y();
        double theta = curr_node_ptr->state_.z();
        
        //steering_radian_step_size_  = steering_radian_ / steering_discrete_num_ = 0.75/3
        const double phi = i * steering_radian_step_size_;

        // forward，segment_length_discrete_num_ 默认是8
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            //move_step_size_ = segment_length/segment_length_discrete_num_ = 1.6/8
            // 利用move_step_size_和phi更新了x,y,theta
            DynamicModel(step_size, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));
            if (!CheckCollision(x, y, theta)) {

                has_obstacle = true;
                break;
            }
        }

        Vec3i grid_index = State2Index(intermediate_state.back());
        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            auto neighbor_forward_node_ptr = new StateNode(grid_index);
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_forward_node_ptr->state_ = intermediate_state.back();
            neighbor_forward_node_ptr->steering_grade_ = i;
            neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;
            neighbor_nodes.push_back(neighbor_forward_node_ptr);
        }

        // backward
        has_obstacle = false;
        intermediate_state.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            DynamicModel(-step_size, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            grid_index = State2Index(intermediate_state.back());
            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->state_ = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
        }
    }
}

void HybridAStar::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) const {
    //这里只是做近似处理
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    // wheel_base_默认是2,是轴距
    // 这里做近似处理，将step_size作为弧长，step_size/R(转向半径）得到车身转向角变化
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}



bool HybridAStar::BeyondBoundary(const Vec2d &pt) const {
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_
     || pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}

double HybridAStar::ComputeH(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &terminal_node_ptr) {
    double h;
    // L2
//    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).norm();

    // L1
    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();
    
    // 当靠近终点时采用RS曲线作为距离
    if (h < 3.0 * shot_distance_) {
        h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
                                   current_node_ptr->state_.z(),
                                   terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
                                   terminal_node_ptr->state_.z());
    }

    return h;
}

double HybridAStar::ComputeReverseH(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &terminal_node_ptr)
{
    double h1 = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();
    double h2 = fabs(target_yaw_ - current_node_ptr->state_[2]);
    // std::cout<<"h1: "<<h1<<", "<<"h2: "<<h2<<std::endl;
    return weight_length_*h1 + weight_yaw_*h2;
}

double HybridAStar::ComputeG(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &neighbor_node_ptr) const {
    double g;
    if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_;
            } else {
                g = segment_length_ * steering_penalty_;
            }
        }
    } else {
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
            }
        } else {
            if (neighbor_node_ptr->steering_grade_ == 0) {
                g = segment_length_ * reversing_penalty_;
            } else {
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }

    return g;
}

bool HybridAStar::ReverseSearch(){

    Vec3d reverse_start_state = goal_state_;
    Vec3d reverse_goal_state = start_state_;

    double stop_search_dist = (reverse_start_state.head(2) - reverse_goal_state.head(2)).lpNorm<2>()/2;

    const Vec3i start_grid_index = State2Index(reverse_start_state);
    const Vec3i goal_grid_index = State2Index(reverse_goal_state);

    auto goal_node_ptr = new StateNode(goal_grid_index);
    goal_node_ptr->state_ = reverse_goal_state;
    goal_node_ptr->direction_ = StateNode::NO;
    goal_node_ptr->steering_grade_ = 0;

    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr->state_ = reverse_start_state;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = StateNode::NO;
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;
    start_node_ptr->intermediate_states_.emplace_back(reverse_start_state); //中间状态？是一个n x 3 的二维向量
    start_node_ptr->g_cost_ = 0.0;
    start_node_ptr->f_cost_ = ComputeReverseH(start_node_ptr, goal_node_ptr);

    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));
    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;

    int count = 0;
    while (!openset_.empty()) {
        //openset使用std::multimap数据结构，其头元素就是当前openset中的代价函数f值最小的节点
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;
        openset_.erase(openset_.begin());

        //认为当搜索节点的角度靠近 target_yaw_，并且距离满足一定的要求stop_search_dist后，认为搜索结束
        if (fabs(current_node_ptr->state_[2] - target_yaw_)<steering_radian_step_size_ 
        && (current_node_ptr->state_.head(2) - start_node_ptr->state_.head(2)).norm() > stop_search_dist) 
        {
            terminal_node_ptr_ = current_node_ptr;
            mid_node_state_ = current_node_ptr->state_;
            std::cout<<"after "<<count<<" times search ,Get the Reverse search goal!"<<std::endl;
            return true;
        }

        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr,reverse_move_step_size_);
         
        //正常搜索过程
        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes_ptr[i];

            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);
            const double current_h = ComputeReverseH(current_node_ptr, goal_node_ptr) * tie_breaker_;

            const Vec3i &index = neighbor_node_ptr->grid_index_;
            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {
                neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + current_h;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
             } 
            else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_OPENSET) {
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;

                if (state_node_map_[index.x()][index.y()][index.z()]->g_cost_ > g_cost_temp) {
                     state_node_map_[index.x()][index.y()][index.z()] -> g_cost_ = g_cost_temp;
                     state_node_map_[index.x()][index.y()][index.z()] -> f_cost_ = g_cost_temp + current_h;
                     state_node_map_[index.x()][index.y()][index.z()] -> parent_node_ = current_node_ptr;
                     state_node_map_[index.x()][index.y()][index.z()] ->intermediate_states_ = neighbor_node_ptr->intermediate_states_;
                     state_node_map_[index.x()][index.y()][index.z()] -> direction_ = neighbor_node_ptr->direction_;
                     state_node_map_[index.x()][index.y()][index.z()]->steering_grade_ = neighbor_node_ptr->steering_grade_;
                } 
                else {
                     delete neighbor_node_ptr;
                }
                continue;
             } 
            else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_CLOSESET) {
                delete neighbor_node_ptr;
                continue;
            }  
        }
        count++;
        if(count %100 == 0)
            ROS_INFO("reverse searched %d times",count);
        if (count > 2000) {
            ROS_WARN("Exceeded the number of iterations, the reverse search failed");
            return false;
        }
    }
    ROS_WARN("Reverse A* search failed after %d times search!",count);
    return false;
}

bool HybridAStar::Search(const Vec3d &mid_node_state ) {
    Timer search_used_time;

    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;

    //将地图中的坐标转换到状态节点图中的索引
    const Vec3i start_grid_index = State2Index(start_state_);
    const Vec3i goal_grid_index = State2Index(mid_node_state);

    auto goal_node_ptr = new StateNode(goal_grid_index);
    goal_node_ptr->state_ = mid_node_state;
    goal_node_ptr->direction_ = StateNode::NO;
    goal_node_ptr->steering_grade_ = 0;

    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr->state_ = start_state_;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = StateNode::NO;
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;
    start_node_ptr->intermediate_states_.emplace_back(start_state_); //中间状态？是一个n x 3 的二维向量
    start_node_ptr->g_cost_ = 0.0;
    start_node_ptr->f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);

    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));
    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;

    int count = 0;
    while (!openset_.empty()) {
        //openset使用std::multimap数据结构，其头元素就是当前openset中的代价函数f值最小的节点
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;
        openset_.erase(openset_.begin());

        //当放入close_set中的节点靠近终点时，认为搜索结束
        if ((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= shot_distance_) {
            ROS_INFO("A* to the goal!");
            double rs_length = 0.0;
            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) {
                terminal_node_ptr_ = goal_node_ptr;

                StateNode::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;
                while (grid_node_ptr != nullptr) {
                    grid_node_ptr = grid_node_ptr->parent_node_;
                    path_length_ = path_length_ + segment_length_;
                }
                path_length_ = path_length_ - segment_length_ + rs_length;

                std::cout << "ComputeH use time(ms): " << compute_h_time << std::endl;
                std::cout << "check collision use time(ms): " << check_collision_use_time << std::endl;
                std::cout << "GetNeighborNodes use time(ms): " << neighbor_time << std::endl;
                std::cout << "average time of check collision(ms): "
                          << check_collision_use_time / num_check_collision
                          << std::endl;
                ROS_INFO("\033[1;32m --> Time in Hybrid A star is %f ms, path length: %f  \033[0m\n",
                         search_used_time.End(), path_length_);

                check_collision_use_time = 0.0;
                num_check_collision = 0.0;
                return true;
            }
        }

        Timer timer_get_neighbor;
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr,move_step_size_);
        neighbor_time = neighbor_time + timer_get_neighbor.End();
         
        //正常搜索过程
        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes_ptr[i];
            Timer timer_compute_g;
            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);
            compute_g_time = compute_g_time + timer_get_neighbor.End();

            Timer timer_compute_h;
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            compute_h_time = compute_h_time + timer_compute_h.End();

            const Vec3i &index = neighbor_node_ptr->grid_index_;
            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {
                neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + current_h;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_OPENSET) {
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;

                if (state_node_map_[index.x()][index.y()][index.z()]->g_cost_ > g_cost_temp) {
                    state_node_map_[index.x()][index.y()][index.z()] -> g_cost_ = g_cost_temp;
                    state_node_map_[index.x()][index.y()][index.z()] -> f_cost_ = g_cost_temp + current_h;
                    state_node_map_[index.x()][index.y()][index.z()] -> parent_node_ = current_node_ptr;
                    delete neighbor_node_ptr;
                } else {
                    delete neighbor_node_ptr;
                }
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_CLOSESET) {
                delete neighbor_node_ptr;
                continue;
            }  
        
        }

        count++;
        if(count %100 ==0)
            ROS_INFO("searched %d times",count);
        if (count > 100000) {
            ROS_WARN("Exceeded the number of iterations, the search failed");
            return false;
        }
    }
    
    ROS_WARN("A* search failed!");
    return false;
}

VectorVec4d HybridAStar::GetSearchedTree() {
    VectorVec4d tree;
    Vec4d point_pair;

    visited_node_number_ = 0;
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k]->parent_node_ == nullptr) {
                    continue;
                }

                const unsigned int number_states = state_node_map_[i][j][k]->intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < number_states; ++l) {
                    point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[l].head(2);
                    point_pair.tail(2) = state_node_map_[i][j][k]->intermediate_states_[l + 1].head(2);

                    tree.emplace_back(point_pair);
                }

                point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[0].head(2);
                point_pair.tail(2) = state_node_map_[i][j][k]->parent_node_->state_.head(2);
                tree.emplace_back(point_pair);
                visited_node_number_++;
            }
        }
    }

    return tree;
}

void HybridAStar::ReleaseMemory() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }

                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }

            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    terminal_node_ptr_ = nullptr;
}

__attribute__((unused)) double HybridAStar::GetPathLength() const {
    return path_length_;
}

VectorVec3d HybridAStar::GetPath() const {
    VectorVec3d path;

    std::vector<StateNode::Ptr> temp_nodes;

    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;
    }

    std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(),
                    node->intermediate_states_.end());
        // path.push_back(node->intermediate_states_.back());
    }

    return path;
}


VectorVec4d HybridAStar::GetPathWithDir() const{
    VectorVec4d path;
    std::vector<StateNode::Ptr> temp_nodes;
    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;

    while(state_grid_node_ptr != nullptr)
    {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr -> parent_node_;
    }

    std::reverse(temp_nodes.begin(),temp_nodes.end());
    for(const auto &node : temp_nodes)
    {
        for(const auto &pt:node->intermediate_states_)
        {
            Eigen::Vector4d temp_pt;
            temp_pt.head(3) = pt;
            if(node->direction_ == StateNode::DIRECTION::FORWARD)
                temp_pt[3] = 1;
            else if(node->direction_ == StateNode::DIRECTION::BACKWARD)
                temp_pt[3] = -1;
            else 
                temp_pt[3] = 0;

            path.emplace_back(temp_pt);
        }

    }

    for (int i = 1;i<path.size();i++)
    {
      if(path.at(i)[3] == 0)
      {
        double tau = atan2(path.at(i)[1] - path.at(i-1)[1], path.at(i)[0] - path.at(i-1)[0]);
        if(fabs(Mod2Pi(path.at(i)[2]) - tau)<M_PI_2)
            path.at(i)[3] = 1;
        else
            path.at(i)[3] = -1;
      }
    }
    for(auto &pt:path)
    {
        pt[2] = Mod2Pi(pt[2]);
    }
    return path;
}


VectorVec4d HybridAStar::GetPathWithDirReverse() const{
    VectorVec4d path;
    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;

   
    while(state_grid_node_ptr != nullptr)
    {
        VectorVec3d inner_states = state_grid_node_ptr->intermediate_states_;
        std::reverse(inner_states.begin(),inner_states.end());

        for(const auto &pt:inner_states)
        {
            Eigen::Vector4d temp_pt;
            temp_pt.head(3) = pt;
            if(state_grid_node_ptr->direction_ == StateNode::DIRECTION::FORWARD)
                temp_pt[3] = -1;
            else if(state_grid_node_ptr->direction_ == StateNode::DIRECTION::BACKWARD)
                temp_pt[3] = 1;
            else 
                temp_pt[3] = 0;
            path.emplace_back(temp_pt);
        }

        state_grid_node_ptr = state_grid_node_ptr -> parent_node_;
    }

    for (int i = 1;i<path.size();i++)
    {
      if(path.at(i)[3] == 0)
      {
        double tau = atan2(path.at(i)[1] - path.at(i-1)[1], path.at(i)[0] - path.at(i-1)[0]);
        if(fabs(Mod2Pi(path.at(i)[2]) - tau)<M_PI_2)
            path.at(i)[3] = 1;
        else
            path.at(i)[3] = -1;
      }
    }
    if(path.at(0)[3] == 0)
       path.at(0)[3] = path.at(1)[3];

    
    for(int i=0;i<path.size()-1;)
    {
        path.at(i)[2] = Mod2Pi(path.at(i)[2]);
        if(path.at(i)[3] != path.at(i+1)[3])
        {  
            path.at(i+1)[3] = path.at(i)[3];
            i = i+2;
        }
        else
            i++;
    }
    path.at(path.size()-1)[2] = Mod2Pi(path.at(path.size()-1)[2]);
    return path;
}



void HybridAStar::Reset() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }

    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}

bool HybridAStar::AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                                     const StateNode::Ptr &goal_node_ptr, double &length) {
    VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->state_,
                                                        goal_node_ptr->state_,
                                                        move_step_size_, length);

    for (const auto &pose: rs_path_poses)
        if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {
            ROS_WARN("AnalyticExpansions failed!");
            return false;
        };

    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);

    return true;
}
