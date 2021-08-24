// Copyright (c) 2021 Trunk Inc. All rights reserved.
#include "src/reeds_shepp_curve.cc"
using namespace util;
int main(int argc, char *argv[]){
  std::cout << "reeds-shepp running "<<std::endl;
  // init points

  Point point{};
  Points _targets;
  point.x = 6.0;
  point.y = -7.0;
  point.theta = 137.4895;
  _targets.emplace_back(point);
  point.x = -6.0;
  point.y = 4.0;
  point.theta = 1.57;
  _targets.emplace_back(point);
  Point basic_vector{};
  Plan::ReedsSheppCurve __rs;
  __rs.Change2Basis(_targets[0], _targets[1],basic_vector);
  std::cout <<basic_vector.x <<"," << basic_vector.y<<","<<basic_vector.theta<< std::endl;
  __rs.GeneratePaths(basic_vector);
  return 0;
}
