
#include "rrt_star_rs/rrt_star.h"

#include <iostream>


RRTStar::RRTStar(const constants &params):params_(params)
{
    vehicle_length_ = params_.vehicle_length + 2*params_.safety_distance;
    vehicle_width_ = params_.vehicle_width+2*params_.safety_distance;

    double front_hang_with_safety_length = params_.front_hang_length + params_.safety_distance;
    double rear_hang_with_safety_length = params_.rear_hang_length + params_.safety_distance;

    SetVehicleShape(front_hang_with_safety_length,rear_hang_with_safety_length, params_.wheel_base, vehicle_width_);

    rs_path_ptr_ = std::make_shared<RSPath>(params_.wheel_base / std::tan(params_.steering_angle));

}

RRTStar::~RRTStar(){
    ReleaseMemory();
}

double RRTStar::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

void RRTStar::ReleaseMemory(){
  for(int i = 0;i<RRTtree.size();i++){
    if(RRTtree[i] != nullptr){
      delete RRTtree[i];
      RRTtree[i] = nullptr;
    }
  }
  root_node_ptr_ = nullptr;
}

void RRTStar::Init(constants &params)
{
    std::vector<double> map_raw_data;
    ReadCsv(params_.csv_path, map_raw_data);
    std::cout<<params_.csv_path<<std::endl;
    
    params_.start_state_original = {map_raw_data[0],map_raw_data[1],Mod2Pi(map_raw_data[2])};
    params_.goal_state_original = {map_raw_data[3],map_raw_data[4],Mod2Pi(map_raw_data[5])};
    
    // regular search 
    goal_state_ = params_.goal_state_original;
    start_state_ = params_.start_state_original;

    root_node_ptr_ = new StateNode(start_state_);
    root_node_ptr_->intermediate_states_.push_back(start_state_);

    int obs_num = static_cast<int>(map_raw_data[6]);
    ROS_INFO("scene has %d obstacles",obs_num);

    int offset = 6+obs_num;

    std::vector<VectorVec2d> obs_array_cardesian;
    std::vector<std::vector<cv::Point2i>> obs_array_grid;

    double low_x = start_state_[0] < goal_state_[0]? start_state_[0]:goal_state_[0];
    double upper_x = start_state_[0] > goal_state_[0]? start_state_[0]:goal_state_[0];
    double low_y = start_state_[1] < goal_state_[1]? start_state_[1]:goal_state_[1];
    double upper_y = start_state_[1] > goal_state_[1]? start_state_[1]:goal_state_[1];

    for(int i=1;i<=obs_num;i++)
    {
        VectorVec2d points_array_cardesian;
        int point_num = map_raw_data[6+i];
        for(int j=1;j<=point_num;++j)
        {
            Vec2d point;
            if(map_raw_data[offset+2*j-1]>upper_x)
                upper_x = map_raw_data[offset+2*j-1];
            else if(map_raw_data[offset+2*j-1]<low_x)
                low_x = map_raw_data[offset+2*j-1];
            if(map_raw_data[offset+2*j]>upper_y)
                upper_y = map_raw_data[offset+2*j];
            else if(map_raw_data[offset+2*j]<low_y)
                low_y = map_raw_data[offset+2*j];

            point[0] = map_raw_data[offset+2*j-1];
            point[1] = map_raw_data[offset+2*j];
            points_array_cardesian.push_back(point);

        }
        obs_array_cardesian.push_back(points_array_cardesian);
        offset += 2*point_num;
    }

    double margin = 5 ;

    params_.map_x_lower = low_x - margin;
    params_.map_x_upper = upper_x + margin;
    params_.map_y_lower = low_y - margin;
    params_.map_y_upper = upper_y + margin;
    

    params_.map_grid_size_x = floor((params_.map_x_upper - params_.map_x_lower) / params_.map_grid_resolution);
    params_.map_grid_size_y = floor((params_.map_y_upper - params_.map_y_lower) / params_.map_grid_resolution);

    
    img_ptr_ = std::make_shared<cv::Mat>(params_.map_grid_size_y ,params_.map_grid_size_x,CV_8UC1,cv::Scalar(0));
    origin_ = {params_.map_x_lower,params_.map_y_lower};
    for(auto &obs:obs_array_cardesian)
    {
        std::vector<cv::Point2i> points_array_grid;
        for(auto &point:obs)
        {
            cv::Point2i point_grid;

            point_grid.x = static_cast<int>((point[0] - origin_[0]) / params_.map_grid_resolution);
            point_grid.y = static_cast<int>((point[1] - origin_[1]) / params_.map_grid_resolution);
            points_array_grid.push_back(point_grid);
        }
        obs_array_grid.push_back(points_array_grid);
    }

    cv::drawContours(*img_ptr_, obs_array_grid, -1, cv::Scalar(255), -1, 8);

    params_.origin = origin_;
    params_.obs_array = obs_array_cardesian;
    params = params_;

}

inline bool RRTStar::LineCheck(double x0, double y0, double x1, double y1) {
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
            if (HasObstacle(Vec2i(yk, x0 + i * 1.0))
                || BeyondBoundary(Vec2d(yk * params_.map_grid_resolution+origin_[0],
                                        (x0 + i) * params_.map_grid_resolution + origin_[1]))
                    ) {
                return false;
            }
        } else {
            if (HasObstacle(Vec2i(x0 + i * 1.0, yk))
                || BeyondBoundary(Vec2d((x0 + i) * params_.map_grid_resolution + origin_[0],
                                        yk * params_.map_grid_resolution + origin_[1]))) {
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

bool RRTStar::CheckCollision(const double &x, const double &y, const double &theta) {
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

bool RRTStar::HasObstacle(const int grid_index_x, const int grid_index_y) const {
    return (grid_index_x >= 0 && grid_index_x < params_.map_grid_size_x
            && grid_index_y >= 0 && grid_index_y < params_.map_grid_size_y
            && (img_ptr_->at<uchar>(grid_index_y,grid_index_x)==255));
}

bool RRTStar::HasObstacle(const Vec2i &grid_index) const {
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < params_.map_grid_size_x
            && grid_index_y >= 0 && grid_index_y < params_.map_grid_size_y
            && (img_ptr_->at<uchar>(grid_index_y,grid_index_x)==255));
}


void RRTStar::SetVehicleShape(double front_hang_length, double rear_hang_length, double wheel_base, double vehicle_width)
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

}


Vec2d RRTStar::MapGridIndex2Coordinate(const Vec2i &grid_index) const {
    Vec2d pt;
    pt.x() = ((double) grid_index[0] + 0.5) * params_.map_grid_resolution + origin_[0];
    pt.y() = ((double) grid_index[1] + 0.5) * params_.map_grid_resolution + origin_[1];

    return pt;
}


Vec2i RRTStar::Coordinate2MapGridIndex(const Vec2d &pt) const {
    Vec2i grid_index;

    grid_index[0] = int((pt[0] - origin_[0]) / params_.map_grid_resolution);
    grid_index[1] = int((pt[1] - origin_[1]) / params_.map_grid_resolution);
    return grid_index;
}

void RRTStar::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) const {
    x = x + step_size * std::cos(theta);
    y = y + step_size * std::sin(theta);
    theta = Mod2Pi(theta + step_size / params_.wheel_base * std::tan(phi));
}


bool RRTStar::BeyondBoundary(const Vec2d &pt) const {
    return pt.x() < params_.map_x_lower || pt.x() > params_.map_x_upper
     || pt.y() < params_.map_y_lower || pt.y() > params_.map_y_upper;
}


Vec3d RRTStar::getRandomPos(){
  Vec2i loc_index ={rand()%params_.map_grid_size_x,rand()%params_.map_grid_size_y};
  Vec2d loc = MapGridIndex2Coordinate(loc_index);
  double phi = static_cast<double>(rand()%628)/100 - 3.14;
  Vec3d pose = {loc[0],loc[1],phi};
  return pose;
}

int RRTStar::getNearestIndex(const Vec3d &pose){
  int closest_index = -1;
  double closest_dist = 1e9;
  for(int i = 0;i<RRTtree.size();i++){
    if(RRTtree[i] != nullptr){
    //   std::cout<<"check whether nullptr exist!"<<std::endl;
      double dist = (RRTtree[i]->state_.head(2) -pose.head(2)).norm();
      if(dist<closest_dist){
        closest_dist = dist;
        closest_index = i;
      }
    }
  }
  return closest_index;
}


StateNode::Ptr RRTStar::steer(const StateNode::Ptr &nearest_node,Vec3d rand_pose){
  double length;
  VectorVec3d rs_path = rs_path_ptr_->GetRSPath(nearest_node->state_,rand_pose,params_.move_step_size,length);
  StateNode::Ptr new_node = new StateNode(rand_pose);
  new_node->intermediate_states_ = rs_path;
  new_node->parent_node_ = nearest_node;
  new_node->g_cost_ = nearest_node->g_cost_ + length;

  auto begin = new_node->intermediate_states_.begin();
  new_node->intermediate_states_.erase(begin);
  return new_node;
}


bool RRTStar::checkPathCollision(const VectorVec3d &path){
    for (const auto &pose: path)
      if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {
        //   ROS_WARN("random node path failed!");
          return true;
      };
    return false;
}

std::vector<int> RRTStar::findNearNodes(const StateNode::Ptr &node){
    int nnode = RRTtree.size() + 1;
    double r = params_.connect_circle_dist * sqrt(log(nnode)/nnode);
    r = std::min(r,params_.expand_dist);
    std::vector<int> result;
    for(int i = 0;i<RRTtree.size();i++){
        if(RRTtree[i] != nullptr){
            if((RRTtree[i]->state_.head(2) - node->state_.head(2)).norm()<=r)
                result.push_back(i);
        }
    }
    return result;
}

void RRTStar::resetParent(StateNode::Ptr &node, const std::vector<int> &indexs){
    if(indexs.empty())
        return ;
    double min_length = 1e9;
    int min_index = -1;
    VectorVec3d min_intermediate_states;
    for(int i = 0;i<indexs.size();i++){
        double length;
        VectorVec3d temp_path = rs_path_ptr_->GetRSPath(RRTtree[indexs[i]]->state_,node->state_,params_.move_step_size,length);
        if(!checkPathCollision(temp_path))
        {
            double new_path_length = RRTtree[indexs[i]]->g_cost_ + length;
            if(new_path_length < min_length){
                min_intermediate_states = temp_path;
                min_length = new_path_length;
                min_index = indexs[i];
            } 
        }
    }
    // ROS_INFO("reselect parent node!");
    min_intermediate_states.erase(min_intermediate_states.begin());
    node->parent_node_ = RRTtree[min_index];
    node->g_cost_ = min_length;
    node->intermediate_states_ = min_intermediate_states;
}


void RRTStar::rewire(const StateNode::Ptr &node, const std::vector<int> &indexs){
    if(indexs.empty())
        return ;
    for(int i = 0;i<indexs.size();i++){
        double length;
        VectorVec3d temp_path = rs_path_ptr_->GetRSPath(node->state_,RRTtree[indexs[i]]->state_,params_.move_step_size,length);
        if(!checkPathCollision(temp_path)){
            double temp_length = node->g_cost_ + length;
            if(temp_length < RRTtree[indexs[i]]->g_cost_){
                // ROS_INFO("rewire!");
                temp_path.erase(temp_path.begin());
                RRTtree[indexs[i]]->parent_node_ = node;
                RRTtree[indexs[i]]->intermediate_states_ = temp_path;
                RRTtree[indexs[i]]->g_cost_ = temp_length;
            }
        }
    }
}

void RRTStar::tryGoalPath(const StateNode::Ptr &node, Timer &used_time,int count){
    double length;
    VectorVec3d temp_path = rs_path_ptr_->GetRSPath(node->state_,goal_state_,params_.move_step_size,length);
    if(!checkPathCollision(temp_path)){
        std::cout<<"try goal path!"<<std::endl;
        temp_path.erase(temp_path.begin());
        StateNode::Ptr goal_node = new StateNode(goal_state_);
        goal_node->g_cost_ = node->g_cost_ + length;
        goal_node->intermediate_states_ = temp_path;
        goal_node->parent_node_ = node;
        goal_node->time_ = used_time.End();
        goal_node->count_ = count;
        RRTtree.emplace_back(goal_node);
    }
}


int RRTStar::searchBestGoalNode(){
    if(RRTtree.empty())
       return -1;
    std::vector<int> best_node_indexs;
    for(int i = 0;i<RRTtree.size();i++){
        if((RRTtree[i]->state_.head(2) - goal_state_.head(2)).norm() < params_.goal_dist_margin 
        && fabs(RRTtree[i]->state_[2] - goal_state_[2])<params_.goal_yaw_margin )
        {
            best_node_indexs.push_back(i);
        }
    }
    if(best_node_indexs.empty())
       return -1;
    double min_cost = 1e9;
    int best_index = -1;
    for(auto j:best_node_indexs){
        if(RRTtree[j]->g_cost_ < min_cost){
            min_cost = RRTtree[j]->g_cost_;
            best_index = j;
        }
    }
    return best_index;
}

int RRTStar::Search(double &time){
  Timer used_time;
  RRTtree.push_back(root_node_ptr_);
  int count = 0;
  tryGoalPath(root_node_ptr_,used_time,count);
  while(count<params_.max_iter){
    std::cout<<"Iter: "<<count<<", number of nodes: "<<RRTtree.size()<<std::endl;
    
    Vec3d rd_pose =  getRandomPos();
    int nearest_index = getNearestIndex(rd_pose);
    StateNode::Ptr new_node = steer(RRTtree[nearest_index],rd_pose);
    if(!checkPathCollision(new_node->intermediate_states_)){
      std::vector<int> near_indexes = findNearNodes(new_node); 
      resetParent(new_node,near_indexes); 
      RRTtree.emplace_back(new_node);
      rewire(new_node,near_indexes);
      tryGoalPath(new_node,used_time,count);
    }
    count++;
  }
  std::cout<<"reached max iteration"<<std::endl;
  time = used_time.End();
  std::cout<<"search used time: "<<time<<std::endl;


  int last_index = searchBestGoalNode();
  if(last_index != -1)
     return last_index;
  return -1;
}


double RRTStar::getBestTime(int best_index) const{
    return RRTtree[best_index]->time_;
}

int RRTStar::getBestCount(int best_index) const{
    return RRTtree[best_index]->count_;
}

VectorVec4d RRTStar::getPath(int best_index ) const{
  std::vector<StateNode::Ptr> node_ptr_lists;
  StateNode::Ptr node_ptr = RRTtree[best_index];
  while(node_ptr != nullptr){
    node_ptr_lists.emplace_back(node_ptr);
    node_ptr = node_ptr->parent_node_;
  }
  std::reverse(node_ptr_lists.begin(),node_ptr_lists.end());
  VectorVec3d path;
  for(auto ptr:node_ptr_lists){
    path.insert(path.end(),ptr->intermediate_states_.begin(),ptr->intermediate_states_.end());
  }
  
  VectorVec4d final_path;
  Vec4d start_point;
  start_point.head(3) = path.at(0);
  start_point[3] = 0;
  final_path.emplace_back(start_point);
  for(int i = 1;i<path.size();i++){
    Vec4d point;
    point.head(3) = path.at(i);
    double tau = atan2(path.at(i)[1] - path.at(i-1)[1], path.at(i)[0] - path.at(i-1)[0]);
    if(fabs(Mod2Pi(path.at(i)[2]) - tau)<M_PI_2)
        point[3] = 1;
    else
        point[3] = -1;
    final_path.emplace_back(point);
  }

  
  return final_path;
}

void RRTStar::GetRRTtree(){
    line_tree.clear();
    Vec4d point_pair;
    for(auto &ptr:RRTtree){
        if(ptr != nullptr && ptr->parent_node_ != nullptr){
            int pt_number = ptr->intermediate_states_.size();
            for(int i = 0;i<pt_number-1;i++){
                point_pair.head(2) = ptr->intermediate_states_[i].head(2);
                point_pair.tail(2) = ptr->intermediate_states_[i+1].head(2);
                line_tree.emplace_back(point_pair);
            }
            point_pair.head(2) = ptr->parent_node_->state_.head(2);
            point_pair.tail(2) = ptr->intermediate_states_[0].head(2);
            line_tree.emplace_back(point_pair);
        }
    }
}

void RRTStar::GetAllPoint(){
    for(auto &ptr:RRTtree){
        if(ptr != nullptr){
            candidate_points.push_back(ptr->state_);
        }
    }
}






