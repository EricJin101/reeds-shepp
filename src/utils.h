// Copyright (c) 2021 Trunk Inc. All rights reserved.
#include <vector>
#include <iostream>

namespace util{

enum STEERING
{
  LEFT = -1,
  RIGHT = 1,
  STRAIGHT = 0
};

enum GEAR
{
  FORWARD = 1,
  BACKWARD = -1
};
class PolarPoint{
public:
  double polar_r;
  double polar_theta;
};

class Point{
public:
  double x;
  double y;
  double theta;
};
class Trajectory{
public:
  STEERING steering;
  GEAR gear;
  double length;
};
typedef std::vector<Point> Points;
typedef std::vector<Trajectory> SingleTrajectory;

}
