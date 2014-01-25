/*
 * edge_pose_pose.cpp
 *
 *  Created on: Jan 19, 2013
 *      Author: ross kidson
 */

// for point transformations
#include "edge_pose_pose.h"

#include "draw_functions.hpp"

#include "math_functions.hpp"

bool EdgePosePose
::write(std::ostream& os) const
{
  Eigen::Quaterniond q(measurement().matrix().topLeftCorner<3,3>()); // extract rotation part
  q.normalize();
  std::vector<double> est(7);
  est[0] = measurement().translation()[0];
  est[1] = measurement().translation()[1];
  est[2] = measurement().translation()[2];
  est[3] = q.x();
  est[4] = q.y();
  est[5] = q.z();
  est[6] = q.w();

  for (int i=0; i<7; i++)
    os << est[i] << " ";

  //information matrix
  for (int i=0; i<information().rows(); i++)
    for (int j=0; j<information().cols(); j++)
      os <<  information()(i,j) << " ";

  return os.good();
}

bool EdgePosePose
::read(std::istream& is)
{
  std::vector<double> est(7);
  for (int i=0; i<7; i++)
    is  >> est[i];

  Eigen::Isometry3d t;
  t= Eigen::Quaterniond(est[6], est[3], est[4], est[5]).toRotationMatrix();
  t.translation() = Eigen::Vector3d(est[0], est[1], est[2]);
  setMeasurement(t);

  if (is.bad()) {
    return false;
  }
  for ( int i=0; i<information().rows() && is.good(); i++)
    for (int j=0; j<information().cols() && is.good(); j++)
      is >> information()(i,j);

  if (is.bad()) {
    //  we overwrite the information matrix with the Identity
    information().setIdentity();
  }

  return true;
}

void EdgePosePose
::computeError()
{
  const VertexPose * w_T_a
      = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose * w_T_b
      = static_cast<const VertexPose*>(_vertices[1]);

  _error = fromIsometry(w_T_a->estimate() * _measurement * w_T_b->estimate().inverse());
}

EdgePosePoseDrawAction::EdgePosePoseDrawAction(): g2o::DrawAction(typeid(EdgePosePose).name()){}

bool EdgePosePoseDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
  if (!DrawAction::refreshPropertyPtrs(params_))
    return false;
  return true;
}

g2o::HyperGraphElementAction* EdgePosePoseDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element,
               HyperGraphElementAction::Parameters* params_){

  if (typeid(*element).name()!=_typeName)
    return 0;

  refreshPropertyPtrs(params_);
  if (! _previousParams)
    return this;

  if (_show && !_show->value())
    return this;

////////////////////////////////////////////////////////////////////////////////////////////////////
/// Get data
  const EdgePosePose* edge_ptr = static_cast<EdgePosePose*>(element);

  const VertexPose * w_T_a
      = static_cast<const VertexPose*>(edge_ptr->vertices()[0]);
  const VertexPose * w_T_b
      = static_cast<const VertexPose*>(edge_ptr->vertices()[1]);

  Eigen::Vector3d vec_a = w_T_a->estimate().translation();
  Eigen::Vector3d vec_b = w_T_b->estimate().translation();

////////////////////////////////////////////////////////////////////////////////////////////////////
/// Draw points

// gl setup
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);

  // draw 3D will draw rays for omni pixels, and 3D points for stereo
  glLineWidth(0.3);
  glColor3f(0.5f,0.5f,1.0f);
  glBegin(GL_LINES);
  glVertex3f(vec_a[0], vec_a[1], vec_a[2]);
  glVertex3f(vec_b[0], vec_b[1], vec_b[2]);
  glEnd();

  // cleanup opengl settings
  glEnable(GL_LIGHTING);
  glPopAttrib();
  return this;
}
