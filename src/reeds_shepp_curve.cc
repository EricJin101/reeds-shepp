// Copyright (c) 2021 Trunk Inc. All rights reserved.

#include "reeds_shepp_curve.h"

namespace Plan {

double ReedsSheppCurve::rad2deg(double rad) {
  return 180 * rad / M_PI;
}

double ReedsSheppCurve::deg2rad(double deg) {
  return M_PI * deg / 180;
}

void ReedsSheppCurve::GeneratePaths(Point point){
//  path_1(point.x, point.y, point.theta);
  path_2(point.x, point.y, point.theta);
  record_path();
  std::cout << "try";
}

Point ReedsSheppCurve::Change2Basis(Point start, Point end, Point& basic_point) {
  /*
  Given p1 = (x1, y1, theta1) and p2 = (x2, y2, theta2) represented in a
  coordinate system with origin (0, 0) and rotation 0 (in degrees), return
      the position and rotation of p2 in the coordinate system which origin
  (x1, y1) and rotation theta1.
  */
  start_point.x = start.x;
  start_point.y = start.y;
  start_point.theta = deg2rad(start.theta);
  double theta1 = deg2rad(start.theta);
  double dx = end.x - start.x;
  double dy = end.y - start.y;
  basic_point.x = dx * cos(theta1) + dy * sin(theta1);
  basic_point.y = -dx * sin(theta1) + dy * cos(theta1);
  basic_point.theta = end.theta - start.theta;
  return basic_point;
}
util::PolarPoint ReedsSheppCurve::CartesianToPolar(double x, double y) {
util::PolarPoint __polar_point{};
__polar_point.polar_r = sqrt(x * x + y * y);
__polar_point.polar_theta = atan2(y, x);
return __polar_point;
}


double ReedsSheppCurve::ModTheta(double theta) {
  theta = fmod(theta, (2 * M_PI));
  if (theta < - M_PI)
    return theta + 2*M_PI;
  if (theta >= M_PI)
    return theta - 2*M_PI;
  return theta;
}

void ReedsSheppCurve::path_1(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  std::cout << "thata: " <<theta<< std::endl;
  double v = ModTheta(__polar_point.polar_theta);
  __polar_point = CartesianToPolar(x - sin(theta), y - 1 + cos(theta));
  std::cout<< "u: "<< __polar_point.polar_r<< " t: "<<__polar_point.polar_theta<<std::endl;

  __path.emplace_back(Create(__polar_point.polar_theta, LEFT, FORWARD));
  __path.emplace_back(Create(__polar_point.polar_r, STRAIGHT, FORWARD));
  __path.emplace_back(Create(v, LEFT, FORWARD));

  paths_.emplace_back(__path);
}

void ReedsSheppCurve::path_2(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  theta = ModTheta(theta);
  __polar_point = CartesianToPolar(x + sin(theta), y - 1 - cos(theta));
  if (__polar_point.polar_r * __polar_point.polar_r > 4){
    double u = sqrt(__polar_point.polar_r * __polar_point.polar_r - 4);
    double t = ModTheta(__polar_point.polar_theta + atan2(2, u));
    double v = ModTheta(t - theta);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(u, STRAIGHT, FORWARD));
    __path.emplace_back(Create(v, RIGHT, FORWARD));
    paths_.emplace_back(__path);
  }
}

void ReedsSheppCurve::path_3(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  theta = ModTheta(theta);
  __polar_point = CartesianToPolar(x + sin(theta), y - 1 - cos(theta));
  if (__polar_point.polar_r * __polar_point.polar_r > 4){
    double u = sqrt(__polar_point.polar_r * __polar_point.polar_r - 4);
    double t = ModTheta(__polar_point.polar_theta + atan2(2, u));
    double v = ModTheta(t - theta);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(u, STRAIGHT, FORWARD));
    __path.emplace_back(Create(v, RIGHT, FORWARD));
    paths_.emplace_back(__path);
  }
}

util::Points ReedsSheppCurve::path_4(double x, double y, double theta) {
  return util::Points();
}

util::Points ReedsSheppCurve::path_5(double x, double y, double theta) {
  return util::Points();
}

util::Points ReedsSheppCurve::path_6(double x, double y, double theta) {
  return util::Points();
}

util::Points ReedsSheppCurve::path_7(double x, double y, double theta) {
  return util::Points();
}

util::Points ReedsSheppCurve::path_8(double x, double y, double theta) {
  return util::Points();
}

util::Points ReedsSheppCurve::path_9(double x, double y, double theta) {
  return util::Points();
}

util::Points ReedsSheppCurve::path_10(double x, double y, double theta) {
  return util::Points();
}

util::Points ReedsSheppCurve::path_11(double x, double y, double theta) {
  return util::Points();
}

util::Points ReedsSheppCurve::path_12(double x, double y, double theta) {
  return util::Points();
}



}