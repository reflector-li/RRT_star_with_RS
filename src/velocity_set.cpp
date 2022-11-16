#include "velocity_set.h"

Trajectory::Trajectory(const constants &params)
    : vel_min_(params.vel_min),
      vel_max_(params.vel_max),
      decel_limit_(params.decel_limit),
      accel_limit_(params.accel_limit),
      radius_thres_(params.radius_thres),
      radius_min_(params.radius_min),
      ang_vel_max_(params.ang_vel_max),
      case_order_(params.case_order) {}


double Trajectory::Mod2Pi(const double &x) {
  double v = fmod(x, 2 * M_PI);

  if (v < -M_PI) {
    v += 2.0 * M_PI;
  } else if (v > M_PI) {
    v -= 2.0 * M_PI;
  }

  return v;
}


void Trajectory::SetPoints(const VectorVec4d &path) {
  points_.clear();
  for (const auto &point : path) {
    waypoint temp(point[0], point[1], point[2], point[3]);
    points_.push_back(temp);
  }
}

//-----------------------------------
// ----- total helpers begin! -----
void Trajectory::SelectCusps(PathSeg &path_segment) {
  int before_cusp_index = 0;
  Dir dir = Dir::UNKNOWN;
  bool unknow_flag = true;
  int unknow_start, unknow_end;
  for (int i = 1; i < points_.size() - 1; i++) {
    Dir cusp_dir = points_.at(i).dir_;
    if (cusp_dir != points_.at(i + 1).dir_) {
      path_segment[before_cusp_index] = std::make_pair(i, cusp_dir);
      before_cusp_index = i;
    }
  }
  path_segment[before_cusp_index] =
      std::make_pair(points_.size() - 1, points_.at(points_.size() - 1).dir_);
}

void Trajectory::LoadPoints(const std::string &fileName) {
  std::ifstream infile(fileName, std::ios::in);
  if (!infile.is_open()) {
    std::cout << "can not open fine" << std::endl;
    return;
  }
  waypoint Point;
  std::string line;
  std::stringstream ss;
  while (getline(infile, line)) {
    ss << line;
    ss >> Point.x_ >> Point.y_;
    Point.yaw_ = 0;
    points_.push_back(Point);
    ss.clear();
  }
  infile.close();
}

void Trajectory::SavePoints() {
  std::string save_path = "/home/linkx/lkx_ws/src/tpcap/Final_path/Traj" +
                          std::to_string(case_order_) + ".csv";
  std::ofstream out_file(save_path, std::ios::trunc);
  for (auto point : update_points_) {
    out_file << std::fixed << std::setprecision(8) << point.time_ << ","
             << point.x_ << "," << point.y_ << "," << point.yaw_ << std::endl;
  }
  out_file.close();
}

void Trajectory::finalCheck(const Vec3d &start_state, const Vec3d &goal_state) {
  if (hypot(update_points_.front().x_ - start_state[0],
            update_points_.front().y_ - start_state[1]) > 0.001 ||
      std::max(fabs(sin(update_points_.front().yaw_) - sin(start_state[2])),
               fabs(cos(update_points_.front().yaw_) - cos(start_state[2]))) >
          0.001)
    ROS_WARN("start point unsatisfied!");

  if (hypot(update_points_.back().x_ - goal_state[0],
            update_points_.back().y_ - goal_state[1]) > 0.001 ||
      std::max(fabs(sin(update_points_.back().yaw_) - sin(goal_state[2])),
               fabs(cos(update_points_.back().yaw_) - cos(goal_state[2]))) >
          0.001)
    ROS_WARN("end point unsatisfied!");

  for (int i = 1; i < update_points_.size(); i++) {
    double delta_t =
        update_points_.at(i).time_ - update_points_.at(i - 1).time_;
    if (delta_t < 0.001 || delta_t > 0.25) ROS_WARN("unsatisfied result!");
  }
}
// ------ total helpers end !-----
// ------------------------------------






// -----------------------------------------------
// ----- resample with curve and traight start !----
const double Trajectory::calcPathLength(
    const std::vector<waypoint> &path) const {
  double distance = 0.0;
  for (unsigned long i = 1; i < path.size(); i++) {
    Eigen::Vector2d pt1(path.at(i - 1).x_, path.at(i - 1).y_);
    Eigen::Vector2d pt2(path.at(i).x_, path.at(i).y_);
    distance += (pt2 - pt1).norm();
  }
  return distance;
}


void Trajectory::calcCurvePara(std::vector<waypoint> &curve_points) {
  if (curve_points.size() != 3) return;
  waypoint &p0 = curve_points.at(0), &p1 = curve_points.at(1),
           &p2 = curve_points.at(2);
  double d = 2 * ((p0.y_ - p2.y_) * (p0.x_ - p1.x_) -
                  (p0.y_ - p1.y_) * (p0.x_ - p2.x_));
  if (fabs(d) < 1e-5) {
    p1.kappa_ = 0;
    std::vector<double> Line = {(p1.x_ - p0.x_), (p1.y_ - p0.y_)};
    p1.dirAngle_ = atan2(Line.at(1), Line.at(0));
    return;
  }
  double a = p0.y_ * p0.y_ - p1.y_ * p1.y_ + p0.x_ * p0.x_ - p1.x_ * p1.x_;
  double b = p0.y_ * p0.y_ - p2.y_ * p2.y_ + p0.x_ * p0.x_ - p2.x_ * p2.x_;

  double cx = ((p0.y_ - p2.y_) * a - (p0.y_ - p1.y_) * b) / d;
  double cy = ((p0.x_ - p2.x_) * a - (p0.x_ - p1.x_) * b) / (-d);

  double dx = cx - p1.x_;
  double dy = cy - p1.y_;
  double R = sqrt(dx * dx + dy * dy);
  p1.kappa_ = 1 / R;
  p0.kappa_ = 1 / R;

  std::vector<double> differVector = {p2.x_ - p1.x_, p2.y_ - p1.y_};
  std::vector<double> dirVec = {-dy / R, dx / R};
  if ((differVector.at(0) * dirVec.at(0) + differVector.at(1) * dirVec.at(1)) <
      0) {
    dirVec.at(0) = -dirVec.at(0);
    dirVec.at(1) = -dirVec.at(1);
  }
  p1.dirAngle_ = atan2(dirVec.at(1), dirVec.at(0));

  // calculate k and dir for p0
  dx = cx - p0.x_;
  dy = cy - p0.y_;
  differVector = {p1.x_ - p0.x_, p1.y_ - p0.y_};
  dirVec = {-dy / R, dx / R};
  if ((differVector.at(0) * dirVec.at(0) + differVector.at(1) * dirVec.at(1)) <
      0) {
    dirVec.at(0) = -dirVec.at(0);
    dirVec.at(1) = -dirVec.at(1);
  }
  p0.dirAngle_ = atan2(dirVec.at(1), dirVec.at(0));
}

void Trajectory::getKappaDir() {
  for (int index = 1; index < points_.size() - 1; index++) {
    waypoint &p0 = points_.at(index - 1), &p1 = points_.at(index),
             &p2 = points_.at(index + 1);
    double d = 2 * ((p0.y_ - p2.y_) * (p0.x_ - p1.x_) -
                    (p0.y_ - p1.y_) * (p0.x_ - p2.x_));
    if (fabs(d) < 1e-8) {
      points_.at(index).kappa_ = 0;
      double module =
          (p2.x_ - p1.x_) * (p2.x_ - p1.x_) + (p2.y_ - p1.y_) * (p2.y_ - p1.y_);
      std::vector<double> Line = {(p2.x_ - p1.x_) / module,
                                  (p2.y_ - p1.y_) / module};
      points_.at(index).dirAngle_ =
          points_.at(index).dir_ == Dir::FORWARD
              ? atan2(Line.at(1), Line.at(0))
              : (atan2(Line.at(1), Line.at(0)) + M_PI);
      points_.at(index).dirAngle_ = Mod2Pi(points_.at(index).dirAngle_);
      continue;
    }
    double a = p0.y_ * p0.y_ - p1.y_ * p1.y_ + p0.x_ * p0.x_ - p1.x_ * p1.x_;
    double b = p0.y_ * p0.y_ - p2.y_ * p2.y_ + p0.x_ * p0.x_ - p2.x_ * p2.x_;

    double cx = ((p0.y_ - p2.y_) * a - (p0.y_ - p1.y_) * b) / d;
    double cy = ((p0.x_ - p2.x_) * a - (p0.x_ - p1.x_) * b) / (-d);

    double dx = cx - p1.x_;
    double dy = cy - p1.y_;
    double R = sqrt(dx * dx + dy * dy);
    points_.at(index).kappa_ = 1 / R;

    std::vector<double> differVector = {p2.x_ - p1.x_, p2.y_ - p1.y_};
    std::vector<double> dirVec = {-dy / R, dx / R};
    if ((differVector.at(0) * dirVec.at(0) +
         differVector.at(1) * dirVec.at(1)) < 0) {
      dirVec.at(0) = -dirVec.at(0);
      dirVec.at(1) = -dirVec.at(1);
    }
    points_.at(index).dirAngle_ =
        points_.at(index).dir_ == Dir::FORWARD
            ? atan2(dirVec.at(1), dirVec.at(0))
            : (atan2(dirVec.at(1), dirVec.at(0)) + M_PI);
    points_.at(index).dirAngle_ = Mod2Pi(points_.at(index).dirAngle_);
  }
  points_.front().dirAngle_ = points_.front().yaw_;
  points_.back().dirAngle_ = points_.back().yaw_;
}

void Trajectory::resampleOnStraight(std::vector<waypoint> &points,
                                    std::vector<waypoint> &curve_points,
                                    double resample_interval) {
  if (curve_points.size() != 3) return;
  waypoint prePoint(points.back());
  prePoint.dir_ = curve_points.at(1).dir_;
  prePoint.yaw_ = curve_points.at(1).yaw_;
  Eigen::Vector2d vecDiff = {curve_points.at(1).x_ - prePoint.x_,
                             curve_points.at(1).y_ - prePoint.y_};
  double dist = vecDiff.norm();
  double coeff = resample_interval / dist;
  vecDiff[0] *= coeff;
  vecDiff[1] *= coeff;
  for (; dist > resample_interval; dist -= resample_interval) {
    prePoint.x_ += vecDiff[0];
    prePoint.y_ += vecDiff[1];
    points.push_back(prePoint);
  }
}

void Trajectory::resampleOnCurve(std::vector<waypoint> &points,
                                 std::vector<waypoint> &curve_points,
                                 double resample_interval) {
  if (curve_points.size() != 3) return;
  waypoint prePoint(curve_points.at(0));
  prePoint.dir_ = curve_points.at(1).dir_;
  prePoint.yaw_ = curve_points.at(1).yaw_;
  double R = 1 / curve_points.at(1).kappa_;
  int dir = 0;

  // judge whither clockwise or anticlockwise
  Eigen::Vector2d p0 = {cos(prePoint.dirAngle_), sin(prePoint.dirAngle_)};
  Eigen::Vector2d p1 = {cos(curve_points.at(1).dirAngle_),
                        sin(curve_points.at(1).dirAngle_)};

  double cross = p0[0] * p1[1] - p0[1] * p1[0];
  double dot = p0.dot(p1);
  double theta = acos(dot / (p0.norm() * p1.norm()));
  if (cross > 0) {
    dir = 1;
  } else {
    dir = -1;  // clockwise
  }

  double dist = fabs(theta) * R;
  double theta_diff = resample_interval * curve_points.at(1).kappa_;
  for (; dist > resample_interval; dist -= resample_interval) {
    if (points.size() == points.capacity()) break;

    Eigen::Vector2d vec = {cos(prePoint.dirAngle_ + dir * theta_diff / 2),
                           sin(prePoint.dirAngle_ + dir * theta_diff / 2)};
    vec = 2 * R * sin(theta_diff / 2) * vec;
    prePoint.dirAngle_ += dir * theta_diff;
    prePoint.x_ += vec[0];
    prePoint.y_ += vec[1];
    points.push_back(prePoint);
  }
}

void Trajectory::resampleWaypoint(const double resample_interval) {
  std::vector<waypoint> temp_final_path;
  PathSeg path_segment;
  SelectCusps(path_segment);
  int original_size = points_.size();
  int start_index = 0;
  while (start_index < original_size - 1) {
    int end_index = path_segment.at(start_index).first;
    std::vector<waypoint> original_path_seg;
    std::vector<waypoint> resample_path_seg;
    original_path_seg.assign(points_.begin() + start_index,
                             points_.begin() + end_index + 1);
    resample_path_seg.push_back(original_path_seg.at(0));
    resample_path_seg.reserve(1.5 * calcPathLength(original_path_seg) /
                              resample_interval);
    for (int i = 1; i < original_path_seg.size(); ++i) {
      std::vector<waypoint> curve_points = {
          resample_path_seg.back(), original_path_seg.at(i),
          i < original_path_seg.size() - 1 ? original_path_seg.at(i + 1)
                                           : original_path_seg.back()};
      calcCurvePara(curve_points);
      if (curve_points.at(1).kappa_ == 0) {
        resampleOnStraight(resample_path_seg, curve_points, resample_interval);
      } else {
        resampleOnCurve(resample_path_seg, curve_points, resample_interval);
      }
    }
    if (end_index == original_size - 1)
      resample_path_seg.push_back(original_path_seg.back());

    if (start_index != 0)
      temp_final_path.insert(temp_final_path.end(),
                             resample_path_seg.begin() + 1,
                             resample_path_seg.end());
    else
      temp_final_path.insert(temp_final_path.end(), resample_path_seg.begin(),
                             resample_path_seg.end());

    start_index = end_index;
  }
  points_.clear();
  points_ = temp_final_path;
  // getKappaDir();
}
// ----- resample with curve and straight end! ---- 
// -----------------------------------------------




// --------------------------------
// ----- visualization  start! ----
void Trajectory::PlotQuiver() {
  std::vector<double> x, y, u, v;
  for (auto &point : points_) {
    x.push_back(point.x_);
    y.push_back(point.y_);
    u.push_back(cos(point.yaw_));
    v.push_back((sin(point.yaw_)));
  }
  plt::quiver(x, y, u, v);
  plt::show();
}


void Trajectory::PlotOriginal(){
    std::vector<double> x, y, x_cusp, y_cusp;
    PathSeg path_segment;
    SelectCusps(path_segment);
    for (auto &point : points_) {
      x.push_back(point.x_);
      y.push_back(point.y_);
    }
    for (auto &seg : path_segment) {
      x_cusp.push_back(points_.at(seg.first).x_);
      y_cusp.push_back(points_.at(seg.first).y_);
      x_cusp.push_back(points_.at(seg.second.first).x_);
      y_cusp.push_back(points_.at(seg.second.first).y_);
    }

    plt::plot(x, y, "r-");
    plt::plot(x, y, "g.");
    plt::plot(x_cusp, y_cusp, "b.");
    plt::show();
}




void Trajectory::Plot() {
  std::vector<double> x, y, x_cusp, y_cusp;
  PathSeg path_segment;
  SelectCusps(path_segment);
  for (auto &point : update_points_) {
    x.push_back(point.x_);
    y.push_back(point.y_);
  }
  for (auto &seg : path_segment) {
    x_cusp.push_back(points_.at(seg.first).x_);
    y_cusp.push_back(points_.at(seg.first).y_);
    x_cusp.push_back(points_.at(seg.second.first).x_);
    y_cusp.push_back(points_.at(seg.second.first).y_);
  }

  plt::plot(x, y, "r-");
  plt::plot(x, y, "g.");
  plt::plot(x_cusp, y_cusp, "b.");

  std::string save_path = "/home/linkx/lkx_ws/src/tpcap/Final_path/Traj" +
                          std::to_string(case_order_) + ".png";
  plt::save(save_path, 300);
  plt::show();
}

void Trajectory::PlotVelocity() {
  std::vector<double> v;
  for (auto &point : update_points_) {
    v.push_back(point.velocity_);
  }
  plt::plot(v, "r.");
  plt::show();
}

// --------------------------------
// ----- visualization end！----



// --------------------------------------
// -------- velocity set start! ----
const CbufGPoint Trajectory::getCrvPoints(int index) const {
  CbufGPoint curve_point(3);
  const int curve_index[3] = {index - 1, index, index + 1};
  for (int i = 0; i < 3; i++) {
    curve_point.push_back(points_.at(curve_index[i]));
  }
  return curve_point;
}

const std::vector<double> Trajectory::calcCurveParam(CbufGPoint p) const {
  for (int i = 0; i < 3; i++,
           p.push_back(p.front()))  // if exception occured, change points order
  {
    const double d = 2 * ((p[0].y_ - p[2].y_) * (p[0].x_ - p[1].x_) -
                          (p[0].y_ - p[1].y_) * (p[0].x_ - p[2].x_));
    if (fabs(d) < 1e-8) {
      continue;
    }
    const std::vector<double> x2 = {p[0].x_ * p[0].x_, p[1].x_ * p[1].x_,
                                    p[2].x_ * p[2].x_};
    const std::vector<double> y2 = {p[0].y_ * p[0].y_, p[1].y_ * p[1].y_,
                                    p[2].y_ * p[2].y_};
    const double a = y2[0] - y2[1] + x2[0] - x2[1];
    const double b = y2[0] - y2[2] + x2[0] - x2[2];
    std::vector<double> param(3);
    const double cx = param[0] =
        ((p[0].y_ - p[2].y_) * a - (p[0].y_ - p[1].y_) * b) / d;
    const double cy = param[1] =
        ((p[0].x_ - p[2].x_) * a - (p[0].x_ - p[1].x_) * b) / -d;
    param[2] =
        sqrt((cx - p[0].x_) * (cx - p[0].x_) + (cy - p[0].y_) * (cy - p[0].y_));
    return param;
  }
  return std::vector<double>();  // error
}

void Trajectory::CreateRadiusList(
    const std::pair<const int, std::pair<int, Dir>> &path,
    std::vector<double> &curve_radius) {
  int size = path.second.first - path.first + 1;
  curve_radius.resize(size);
  curve_radius.at(0) = curve_radius.back() = 10 * radius_thres_;

  // 需改进，当每个分段的点数为2个时，这个部分会出问题
  for (int i = 1; i < size - 1; i++) {
    CbufGPoint curve_point(getCrvPoints(path.first + i));
    const std::vector<double> curve_param(calcCurveParam(curve_point));

    // if going straight
    if (curve_param.empty()) {
      curve_radius.at(i) = 10 * radius_thres_;
    }
    // else if turnning curve
    else {
      curve_radius.at(i) = (curve_param[2] > 10 * radius_thres_)
                               ? 10 * radius_thres_
                               : curve_param[2];
    }
  }
}

void Trajectory::CreateCurveList(
    const std::pair<const int, std::pair<int, Dir>> &path,
    const std::vector<double> &curve_radius, KeyVal &curve_list) {
  int index = 0;
  bool on_curve = false;
  double radius_localmin = DBL_MAX;
  for (int i = 1; i < curve_radius.size(); i++) {
    if (!on_curve && curve_radius[i] <= radius_thres_ &&
        curve_radius[i - 1] > radius_thres_) {
      index = i + path.first;
      on_curve = true;
    } else if (on_curve && curve_radius[i - 1] <= radius_thres_ &&
               curve_radius[i] > radius_thres_) {
      on_curve = false;
      if (radius_localmin < radius_min_) {
        radius_localmin = radius_min_;
      }
      curve_list[index] = std::make_pair(i + path.first, radius_localmin);
      radius_localmin = DBL_MAX;
    }
    if (!on_curve) {
      continue;
    }
    if (radius_localmin > curve_radius[i]) {
      radius_localmin = curve_radius[i];
    }
  }
}

void Trajectory::LimitVelocityByRange(
    int start_index, int end_index,
    const std::pair<const int, std::pair<int, Dir>> &path, double vmin) {
  for (int idx = start_index; idx <= end_index; idx++) {
    if (points_.at(idx).velocity_ < vmin) {
      continue;
    }
    points_.at(idx).velocity_ = vmin;
  }
  limitAccelDecel(start_index, path);
  limitAccelDecel(end_index, path);
}

void Trajectory::limitAccelDecel(
    int idx, const std::pair<const int, std::pair<int, Dir>> &path) {
  double vel_limit = path.second.second == Dir::FORWARD ? vel_max_ : vel_min_;
  const double acc[2] = {accel_limit_, decel_limit_};
  const int end_idx[2] = {path.second.first - idx, idx - path.first};
  const int sgn[2] = {1, -1};
  for (int j = 0; j < 2; j++)  // [j=0]: config_.accel_limitprocess, [j=1]:
                               // config_.decel_limitprocess
  {
    double v = points_.at(idx).velocity_;
    int next = idx + sgn[j];
    for (int i = 1; i <= end_idx[j]; i++, next += sgn[j]) {
      const waypoint &p0 = points_.at(next - sgn[j]);
      const waypoint &p1 = points_.at(next);
      const double dist = std::hypot(p0.x_ - p1.x_, p0.y_ - p1.y_);
      v = sqrt(2 * acc[j] * dist + v * v);
      if (v > vel_limit || v > points_.at(next).velocity_) {
        break;
      }
      points_.at(next).velocity_ = v;
    }
  }
}

void Trajectory::InitVelocity(PathSeg &path_segment) {
  for (const auto &path : path_segment) {
    double vel_limit = path.second.second == Dir::FORWARD ? vel_max_ : vel_min_;
    points_.at(path.first).velocity_ = 0;
    points_.at(path.second.first).velocity_ = 0;
    for (int i = path.first + 1; i < path.second.first; ++i) {
      points_.at(i).velocity_ = vel_limit;
    }
  }
}

void Trajectory::changeVelSign(
    std::pair<const int, std::pair<int, Dir>> &path) {
  for (int i = path.first; i < path.second.first; ++i) {
    int sgn = path.second.second == Dir::FORWARD ? 1 : -1;
    points_.at(i).velocity_ = sgn * fabs(points_.at(i).velocity_);
  }
}

void Trajectory::SetTime() {
  points_.at(0).time_ = 0;
  waypoint front_pt = points_.at(0);
  update_points_.emplace_back(front_pt);
  for (int i = 1; i < points_.size(); ++i) {
    waypoint p1 = points_.at(i);

    double v = (front_pt.velocity_ + p1.velocity_) / 2;
    if (fabs(v) < 1e-2) v = 0.3;
    double dist = std::hypot(p1.x_ - front_pt.x_, p1.y_ - front_pt.y_);
    double delta_t = dist / fabs(v);
    if (delta_t > 0.25) {
      Eigen::Vector2d dir;
      dir << p1.x_ - front_pt.x_, p1.y_ - front_pt.y_;
      dir = dir / dir.norm();
      int interp_num = floor(delta_t / 0.25);
      double seg_dist = dist / (interp_num + 1);
      double seg_yaw = (p1.yaw_ - front_pt.yaw_) / (interp_num + 1);
      double seg_delta_t = (delta_t) / (interp_num + 1);
      for (int j = 0; j < interp_num; j++) {
        waypoint interp_pt;
        interp_pt.x_ = front_pt.x_ + (j + 1) * seg_dist * dir[0];
        interp_pt.y_ = front_pt.y_ + (j + 1) * seg_dist * dir[1];
        interp_pt.dir_ = front_pt.dir_;
        interp_pt.velocity_ = v;
        interp_pt.yaw_ = front_pt.yaw_ + (j + 1) * seg_yaw;
        interp_pt.time_ = front_pt.time_ + (j + 1) * seg_delta_t;
        update_points_.emplace_back(interp_pt);
      }
    } else if (delta_t < 0.001)
      continue;

    p1.time_ = front_pt.time_ + delta_t;
    update_points_.emplace_back(p1);
    front_pt = p1;
  }
}

void Trajectory::SetVelocity() {
  PathSeg path_segment;
  SelectCusps(path_segment);
  InitVelocity(path_segment);
  // PlotVelocity();
  for (auto &path : path_segment) {
    double v_min = path.second.second == Dir::FORWARD ? vel_max_ : vel_min_;
    LimitVelocityByRange(path.first, path.second.first, path, v_min);
    std::vector<double> curve_radius;
    KeyVal curve_list;
    CreateRadiusList(path, curve_radius);
    CreateCurveList(path, curve_radius, curve_list);
    for (const auto &el : curve_list) {
      const double &radius = el.second.second;
      double vmin = ang_vel_max_ * radius;
      LimitVelocityByRange(el.first, el.second.first, path, vmin);
    }
    changeVelSign(path);
  }
  SetTime();
}

// -------- velocity set end! --------
// ---------------------------------------------

