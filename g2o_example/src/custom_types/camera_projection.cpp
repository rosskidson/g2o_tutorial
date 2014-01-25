/*
 * uvm_projection.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: ross kidson
 */

#include "camera_projection.h"

#include <iostream>

bool CameraProjection
::read(std::istream& is)
{
  K_ = Eigen::Matrix3d::Identity();
  double f_x, f_y, c_x, c_y;
  is >> f_x;
  is >> f_y;
  is >> c_x;
  is >> c_y;

  //populate K with these values
  K_(0,0) = f_x;
  K_(1,1) = f_y;
  K_(0,2) = c_x;
  K_(1,2) = c_y;
  K_(2,2) = 1.0;

  if(K_(0,0) == 0.0 || K_(1,1) == 0.0) //check K is populated
  {
    std::cout << " error loading projection pointer\n";
    return false;
  }

  return true;
}

bool CameraProjection
::write(std::ostream& os) const
{
  double f_x, f_y, c_x, c_y;
  f_x = K_(0,0);
  f_y = K_(1,1);
  c_x = K_(0,2);
  c_y = K_(1,2);
  os << f_x << " ";
  os << f_y << " ";
  os << c_x << " ";
  os << c_y << " ";

  return true;
}

Eigen::Vector2d CameraProjection
::cam_map(const Eigen::Vector3d & P_cam) const
{
  Eigen::Vector3d P_normalized;
  P_normalized[0] = P_cam[0]/P_cam[2];
  P_normalized[1] = P_cam[1]/P_cam[2];
  P_normalized[2] = 1;
  Eigen::Vector3d P_img = K_ * P_normalized;

  return P_img.block<2,1>(0,0);
}

Eigen::Vector3d CameraProjection
::ray_map(const Eigen::Vector2d & P_img) const
{
  Eigen::Vector3d P_ray;
  P_ray = K_.inverse() * Eigen::Vector3d(P_img[0], P_img[1], 1);
  return P_ray;
}
