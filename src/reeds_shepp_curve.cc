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
  paths_.emplace_back(path_1(point.x, point.y, point.theta));
  paths_.emplace_back(path_2(point.x, point.y, point.theta));
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

SingleTrajectory ReedsSheppCurve::path_1(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  std::cout << "theta: " <<theta<< std::endl;
  double v = ModTheta(__polar_point.polar_theta);
  __polar_point = CartesianToPolar(x - sin(theta), y - 1 + cos(theta));
  std::cout<< "u: "<< __polar_point.polar_r<< " t: "<<__polar_point.polar_theta<<std::endl;

  __path.emplace_back(Create(__polar_point.polar_theta, LEFT, FORWARD));
  __path.emplace_back(Create(__polar_point.polar_r, STRAIGHT, FORWARD));
  __path.emplace_back(Create(v, LEFT, FORWARD));

  return __path;
}

SingleTrajectory ReedsSheppCurve::path_2(double x, double y, double theta) {
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
  }
  return __path;
}

SingleTrajectory ReedsSheppCurve::path_3(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  double xi = x - sin(theta);
  double eta = y - 1 + cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
  if (__polar_point.polar_r <= 4){
    double A = acos(__polar_point.polar_r / 4);
    double t = ModTheta(theta + M_PI_2 + A);
    double u = ModTheta(M_PI - 2 * A);
    double v = ModTheta(theta - t - u);
    
    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(u, RIGHT, BACKWARD));
    __path.emplace_back(Create(v, LEFT, FORWARD));
  }
  return __path;
}

SingleTrajectory ReedsSheppCurve::path_4(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  double xi = x - sin(theta);
  double eta = y - 1 + cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
  if (__polar_point.polar_r <= 4){
    double A = acos(__polar_point.polar_r / 4);
    double t = ModTheta(theta + M_PI_2 + A);
    double u = ModTheta(M_PI - 2 * A);
    double v = ModTheta(t + u - theta);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(u, RIGHT, BACKWARD));
    __path.emplace_back(Create(v, LEFT, BACKWARD));
  }
  return __path;
}

SingleTrajectory ReedsSheppCurve::path_5(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  double xi = x - sin(theta);
  double eta = y - 1 + cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
  if (__polar_point.polar_r <= 4){
    double u = acos(1 - __polar_point.polar_r * __polar_point.polar_r / 8);
    double A = asin(2 * sin(u) / __polar_point.polar_r);
    double t = ModTheta(theta + M_PI_2 - A);
    double v = ModTheta(t - u - theta);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(u, RIGHT, FORWARD));
    __path.emplace_back(Create(v, LEFT, BACKWARD));
  }
  return __path;
}

SingleTrajectory ReedsSheppCurve::path_6(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  double xi = x + sin(theta);
  double eta = y - 1 - cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
  double A, t, u, v;
  if (__polar_point.polar_r <= 4){
    if (__polar_point.polar_r <= 2){
      A = acos((__polar_point.polar_r + 2) / 4);
      t = ModTheta(theta + M_PI_2 + A);
      u = ModTheta(A);
      v = ModTheta(theta - t + 2 * u);
    }else{
      A = acos((__polar_point.polar_r - 2) / 4);
      t = ModTheta(theta + M_PI_2 - A);
      u = ModTheta(M_PI - A);
      v = ModTheta(theta - t + 2 * u);
    }

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(u, RIGHT, FORWARD));
    __path.emplace_back(Create(u, LEFT, BACKWARD));
    __path.emplace_back(Create(v, RIGHT, FORWARD));
  }
  return __path;
} 
SingleTrajectory ReedsSheppCurve::path_7(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  
  double xi = x + sin(theta);
  double eta = y - 1 - cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
  double u1 = (20 - __polar_point.polar_r * __polar_point.polar_r) / 16;

  if(__polar_point.polar_r <= 6 && 0 <= u1 <= 1){
    double u = acos(u1);
    double A = asin(2 * sin(u) / __polar_point.polar_r);
    double t = ModTheta(theta + M_PI_2 + A);
    double v = ModTheta(t - theta);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(u, RIGHT, BACKWARD));
    __path.emplace_back(Create(u, LEFT, BACKWARD));
    __path.emplace_back(Create(u, RIGHT, FORWARD));
  }
  return __path;
}

SingleTrajectory ReedsSheppCurve::path_8(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  
  double xi = x - sin(theta);
  double eta = y - 1 + cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
  if (__polar_point.polar_r >= 2){
    double u = sqrt(__polar_point.polar_r * __polar_point.polar_r - 4) - 2;
    double A = atan2(2, u + 2);
    double t = ModTheta(theta + M_PI_2 + A);
    double v = ModTheta(t - theta + M_PI_2);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(M_PI_2, RIGHT, BACKWARD));
    __path.emplace_back(Create(u, STRAIGHT, BACKWARD));
    __path.emplace_back(Create(v, LEFT, BACKWARD));
  }
  return __path;
}

SingleTrajectory ReedsSheppCurve::path_9(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  
  double xi = x - sin(theta);
  double eta = y - 1 + cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
    
  if (__polar_point.polar_r >= 2){
    double u = sqrt(__polar_point.polar_r * __polar_point.polar_r - 4) - 2;
    double A = atan2(u + 2, 2);
    double t = ModTheta(theta + M_PI_2 - A);
    double v = ModTheta(t - theta - M_PI_2);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(u, STRAIGHT, FORWARD));
    __path.emplace_back(Create(M_PI_2, RIGHT, FORWARD));
    __path.emplace_back(Create(v, LEFT, BACKWARD));
  }
  return __path;
}

SingleTrajectory ReedsSheppCurve::path_10(double x, double y, double theta) {
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  
  double xi = x - sin(theta);
  double eta = y - 1 + cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
    
  if (__polar_point.polar_r >= 2){
    double t = ModTheta(theta + M_PI_2);
    double u = __polar_point.polar_r - 2;
    double v = ModTheta(theta - t - M_PI_2);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(M_PI_2, RIGHT, BACKWARD));
    __path.emplace_back(Create(u, STRAIGHT, BACKWARD));
    __path.emplace_back(Create(v, RIGHT, BACKWARD));
  }
  return __path;
}

SingleTrajectory ReedsSheppCurve::path_11(double x, double y, double theta) {
  
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  
  double xi = x + sin(theta);
  double eta = y - 1 - cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
    
  if (__polar_point.polar_r >= 2){
    double t = ModTheta(theta);
    double u = __polar_point.polar_r - 2;
    double v = ModTheta(theta - t - M_PI_2);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(u, STRAIGHT, FORWARD));
    __path.emplace_back(Create(M_PI_2, LEFT, FORWARD));
    __path.emplace_back(Create(v, RIGHT, BACKWARD));
  }
  return __path;
}

SingleTrajectory ReedsSheppCurve::path_12(double x, double y, double theta) {
  
  std::vector<Trajectory> __path;
  PolarPoint __polar_point{};
  theta = deg2rad(theta);
  
  double xi = x + sin(theta);
  double eta = y - 1 - cos(theta);
  __polar_point = CartesianToPolar(xi, eta);
    
  if (__polar_point.polar_r >= 4){
    double u = sqrt(__polar_point.polar_r * __polar_point.polar_r - 4) - 4;
    double A = atan2(2, u + 4);
    double t = ModTheta(theta + M_PI_2 + A);
    double v = ModTheta(t - theta);

    __path.emplace_back(Create(t, LEFT, FORWARD));
    __path.emplace_back(Create(M_PI_2, RIGHT, FORWARD));
    __path.emplace_back(Create(u, STRAIGHT, BACKWARD)); 
    __path.emplace_back(Create(u, STRAIGHT, BACKWARD));
    __path.emplace_back(Create(v, RIGHT, FORWARD));
  }
  return __path;
  
}

std::vector<Trajectory> ReedsSheppCurve::time_flipping(double x, double y, double theta) {
  return std::vector<Trajectory>();
}

std::vector<Trajectory> ReedsSheppCurve::reflecting(double x, double y, double theta) {
  return std::vector<Trajectory>();
}


}
