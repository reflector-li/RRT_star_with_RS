#ifndef _VELOCITY_SET_H_
#define _VELOCITY_SET_H_

#include <float.h>
#include <ros/ros.h>

#include <boost/circular_buffer.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

#include "rrt_star_rs/type.h"
#include "matplotlibcpp.h"
#include "constants.h"

namespace plt = matplotlibcpp;
typedef std::unordered_map<int, std::pair<int, double>> KeyVal;

enum class Dir { FORWARD = 1, BACKWARD = -1, UNKNOWN = 0 };

struct waypoint {
 private:
  /* data */
 public:
  double x_, y_, yaw_;
  double velocity_;
  double kappa_;
  double dirAngle_;
  Dir dir_;  // 0 for forward, 1 for backward, 2 for unknow;
  double time_;
  waypoint() = default;
  waypoint(double x, double y, double yaw, int dir) : x_(x), y_(y), yaw_(yaw) {
    if (dir == 1)
      dir_ = Dir::FORWARD;
    else if (dir == -1)
      dir_ = Dir::BACKWARD;
    else
      dir_ = Dir::UNKNOWN;
    velocity_ = 0;
    time_ = 0;
    kappa_ = 0;
    dirAngle_ = 0;
  };
  ~waypoint(){};
};

typedef boost::circular_buffer<waypoint> CbufGPoint;

/* PathSeg to distinguish different
   segment of whole path divided by cusps.
*/
typedef std::unordered_map<int, std::pair<int, Dir>> PathSeg;

class Trajectory {
 private:
  void SelectCusps(PathSeg &path_segment);
  void InitVelocity(PathSeg &path_segment);
  void LimitVelocityByRange(
      int start_index, int end_index,
      const std::pair<const int, std::pair<int, Dir>> &path, double vmin);
  void limitAccelDecel(int idx,
                       const std::pair<const int, std::pair<int, Dir>> &path);
  const CbufGPoint getCrvPoints(int index) const;
  const std::vector<double> calcCurveParam(CbufGPoint p) const;
  void CreateRadiusList(const std::pair<const int, std::pair<int, Dir>> &path,
                        std::vector<double> &curve_radius);
  void CreateCurveList(const std::pair<const int, std::pair<int, Dir>> &path,
                       const std::vector<double> &curve_radius,
                       KeyVal &curve_list);
  void changeVelSign(std::pair<const int, std::pair<int, Dir>> &path);
  void SetTime();
  void calcCurvePara(std::vector<waypoint> &curve_points);
  void resampleOnStraight(std::vector<waypoint> &vecTraj,std::vector<waypoint> &curve_points,double resample_interval);
  void resampleOnCurve(std::vector<waypoint> &points,std::vector<waypoint> &curve_points,double resample_interval);

 public:
  std::vector<waypoint> points_;
  std::vector<waypoint> update_points_;
  Dir dir_;
  double vel_min_;
  double vel_max_;
  double decel_limit_;  // positive number
  double accel_limit_;  // positive number
  double radius_thres_;
  double radius_min_;
  double ang_vel_max_;
  int case_order_;

  Trajectory() = default;
  Trajectory(const constants &params);
  ~Trajectory(){};
  void SetPoints(const VectorVec4d &path);

  //for resample 
  void resampleWaypoint(const double resample_interval);
  const double calcPathLength(const std::vector<waypoint> &path) const;
  void getKappaDir();
  double Mod2Pi(const double &x);
  void PlotQuiver();
  void Plot();
  void PlotOriginal();
  void SetVelocity();

  // for debug
  void PlotVelocity();
  void LoadPoints(const std::string &fileName);
  void SavePoints();
  void finalCheck(const Vec3d &start_state, const Vec3d &goal_state);
  
};

#endif