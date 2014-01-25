/*
 * camera_projection.h
 *
 *  Created on: Jan 19, 2014
 *      Author: ross kidson
 */

#ifndef CAMERA_PROJECTION_H_
#define CAMERA_PROJECTION_H_

#include <g2o/core/parameter.h>

#include <Eigen/Core>
#include <Eigen/LU>

class CameraProjection : public g2o::Parameter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraProjection():
    K_(Eigen::Matrix3d::Identity())
  {}

  CameraProjection(const double f_x, const double f_y, const double c_x, const double c_y):
    K_(Eigen::Matrix3d::Identity())
  {
    //populate K with these values
    K_(0,0) = f_x;
    K_(1,1) = f_y;
    K_(0,2) = c_x;
    K_(1,2) = c_y;
    K_(2,2) = 1.0;
  }

  Eigen::Vector2d
  cam_map                    (const Eigen::Vector3d & P_cam) const;

  Eigen::Vector3d
  ray_map                    (const Eigen::Vector2d & P_img) const;

  virtual bool
  read                       (std::istream& is);

  virtual bool
  write                      (std::ostream& os) const;

  Eigen::Matrix3d K_;
};

#endif
