// Copyright (c) 2021 Trunk Inc. All rights reserved.
#include "utils.h"
#include <cmath>

using namespace util;
namespace Plan {
class ReedsSheppCurve {

public:


  PolarPoint CartesianToPolar(double x, double y);
  Point Change2Basis(Point start, Point end, Point& basic_point);
  void GeneratePaths(Point point);
  double ModTheta(double theta);

  double rad2deg(double rad);
  double deg2rad(double deg);


  SingleTrajectory path_1(double x, double y, double theta);
  SingleTrajectory path_2(double x, double y, double theta);
  SingleTrajectory path_3(double x, double y, double theta);
  SingleTrajectory path_4(double x, double y, double theta);
  SingleTrajectory path_5(double x, double y, double theta);
  SingleTrajectory path_6(double x, double y, double theta);
  SingleTrajectory path_7(double x, double y, double theta);
  SingleTrajectory path_8(double x, double y, double theta);
  SingleTrajectory path_9(double x, double y, double theta);
  SingleTrajectory path_10(double x, double y, double theta);
  SingleTrajectory path_11(double x, double y, double theta);
  SingleTrajectory path_12(double x, double y, double theta);
  SingleTrajectory time_flipping(double x, double y, double theta);
  SingleTrajectory reflecting(double x, double y, double theta);


protected:
  Point start_point{};
  std::vector<Trajectory> path_;
  std::vector<std::vector<Trajectory>> paths_;

protected:
  Trajectory Create(double param, STEERING steering, GEAR gear){
    Trajectory _traj{};
    _traj.steering = steering;
    if (_traj.length >= 0){
      _traj.length = param;
      _traj.gear = gear;
      return _traj;
    }else{
      _traj.length = -param;
      _traj.gear = static_cast<GEAR>(-gear);
      return _traj;
    }
  }

  Trajectory ReverseGear(Trajectory _traj){
    _traj.gear = static_cast<GEAR>(-_traj.gear);
    return _traj;
  }

  Trajectory ReverseSteering(Trajectory _traj){
    _traj.steering = static_cast<STEERING >(-_traj.steering);
    return _traj;
  }

  void record_path(){

    std::vector<Point> final_pts;
    Point __point{};
    for (const auto& path : paths_){
      double X = start_point.x;
      double Y = start_point.y;
      double _the = start_point.theta;
      for (auto p : path){

        if (p.steering == LEFT){
          double r_x = 0.0;
          double r_y = 0.0;
          r_x = X - sin(start_point.theta);
          r_y = Y + cos(start_point.theta);  // 圆心坐标
          double _len = fabs(p.length);

          if (p.gear == FORWARD){
            for (double _delta = 0.0; _delta < _len - 0.1; ){
              double __x = r_x + cos(_the + 0.1 - M_PI_2);
              double __y = r_y + sin(_the + 0.1 - M_PI_2);
              X = __x;
              Y = __y;
              __point.x = __x;
              __point.y = __y;
              std::cout <<"x:"<<__x<<"y:"<<__y<<std::endl;
              final_pts.emplace_back(__point);
              _delta += 0.1;
            }
          }else{
            for (double _delta = 0.0; _delta < _len - 0.1; ){
              double __x = r_x + cos(_the - 0.1 - M_PI_2);
              double __y = r_y + sin(_the - 0.1 - M_PI_2);
              X = __x;
              Y = __y;
              __point.x = __x;
              __point.y = __y;
              std::cout <<"x:"<<__x<<"y:"<<__y<<std::endl;
              final_pts.emplace_back(__point);
              _delta += 0.1;
            }
          }
        }else if(p.steering == STRAIGHT){
          double _len = fabs(p.length);
          double _l =0.0;
          if (_len >0) {
            _l = 0.1;
          }else{
            _l = -0.1;
          }

          for (double _delta = 0.0; fabs(_delta) < fabs(_len) - 0.1; ){
            X += 0.1 * cos(_the);
            Y += 0.1 * sin(_the);
            __point.x = X;
            __point.y = Y;
            std::cout <<"x:"<<X<<"y:"<<Y<<std::endl;
            final_pts.emplace_back(__point);
            _delta += _l;
          }
        }else{// right
          double r_x = 0.0;
          double r_y = 0.0;
          r_x = X + sin(start_point.theta);
          r_y = Y - cos(start_point.theta);  // 圆心坐标
          double _len = fabs(p.length);

          if (p.gear == FORWARD){
            for (double _delta = 0.0; fabs(_delta) < fabs(_len) - 0.1; ){
              double __x = r_x - sin(_the - _delta + M_PI_2);
              double __y = r_y - cos(_the - _delta + M_PI_2);
              X = __x;
              Y = __y;
              __point.x = __x;
              __point.y = __y;
              std::cout <<"x:"<<__x<<"y:"<<__y<<std::endl;
              final_pts.emplace_back(__point);
              _delta += 0.1;
            }

          }
        }

      }

    }
  }

};

}
