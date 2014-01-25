
#ifndef MATH_FUNCTIONS_HPP
#define MATH_FUNCTIONS_HPP

#include <Eigen/Core>

  // converts compact quaternion (3D vector part of quaternion) to a rotation matrix
  inline Eigen::Matrix3d fromCompactQuat(const Eigen::Vector3d& v) {
    double w = 1-v.squaredNorm();
    if (w<0)
      return Eigen::Matrix3d::Identity();
    else
      w=sqrt(w);
    return Eigen::Quaterniond(w, v[0], v[1], v[2]).toRotationMatrix();
  }

  // converts from a 6D vector to a SE(3) Isometry transform
  inline Eigen::Isometry3d fromVectorQuat(const Eigen::Matrix<double, 6, 1>& v){
    Eigen::Isometry3d T;
    T = fromCompactQuat(v.block<3,1>(3,0));
    T.translation() = v.block<3,1>(0,0);
    return T;
  }

  // converts from a SE(3) Isometry transform to a 6D vector
  inline Eigen::Matrix<double, 6, 1> fromIsometry(const Eigen::Isometry3d & T){
    Eigen::Matrix<double, 6, 1> v;
    v.block<3,1>(0,0) = T.translation();

    Eigen::Quaterniond q(T.rotation());
    q.normalize();
    v.block<3,1>(3,0) = q.vec();

    return v;
  }

  /**
  * compute a fast approximation for the nearest orthogonal rotation matrix.
  * The function computes the residual E = RR^T - I which is then used as follows:
  * R := R - 1/2 R E
  */
  template <typename Derived>
  inline void approximateNearestOrthogonalMatrix(const Eigen::MatrixBase<Derived>& R)
  {
    Eigen::Matrix3d E = R.transpose() * R;
    E.diagonal().array() -= 1;
    const_cast<Eigen::MatrixBase<Derived>&>(R) -= 0.5 * R * E;
  }

#endif
