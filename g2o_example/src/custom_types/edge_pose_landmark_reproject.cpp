/*
 * edge_pose_landmark_reproject.cpp
 *
 *  Created on: Jan 19, 2013
 *      Author: ross kidson
 */

// for point transformations
#include "edge_pose_landmark_reproject.h"

#include "draw_functions.hpp"

bool EdgePoseLandmarkReproject
::write(std::ostream& os) const
{
  os  << measurement()[0] << " "
      << measurement()[1] << " ";

  //information matrix
  for (int i=0; i<information().rows(); i++)
    for (int j=0; j<information().cols(); j++)
      os <<  information()(i,j) << " ";

  return true;
}

bool EdgePoseLandmarkReproject
::read(std::istream& is)
{
  double u, v;
  is >> u;
  is >> v;
  setMeasurement(Eigen::Vector2d(u,v));

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

void EdgePoseLandmarkReproject
::computeError()
{
  const VertexPose * w_T_cam
      = static_cast<const VertexPose*>(_vertices[0]);
  const VertexLandmarkXYZ * P_world
      = static_cast<const VertexLandmarkXYZ*>(_vertices[1]);

  //std::cout << "meas " << _measurement << " graph " << (w_T_cam->estimate().inverse() * P_world->estimate()) << "\n";
  _error = _measurement - cam->cam_map(w_T_cam->estimate().inverse() * P_world->estimate());
}

EdgePoseLandmarkReprojectDrawAction::EdgePoseLandmarkReprojectDrawAction(): g2o::DrawAction(typeid(EdgePoseLandmarkReproject).name()){}

bool EdgePoseLandmarkReprojectDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
  if (!DrawAction::refreshPropertyPtrs(params_))
    return false;
  return true;
}

g2o::HyperGraphElementAction* EdgePoseLandmarkReprojectDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element,
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
  const EdgePoseLandmarkReproject* reproj_edge_ptr = static_cast<EdgePoseLandmarkReproject*>(element);

  const VertexPose * w_T_cam
      = static_cast<const VertexPose*>(reproj_edge_ptr->vertices()[0]);
  const VertexLandmarkXYZ * P_world
      = static_cast<const VertexLandmarkXYZ*>(reproj_edge_ptr->vertices()[1]);

  Eigen::Vector3d camera = w_T_cam->estimate().translation();
  Eigen::Vector3d landmark = P_world->estimate();

////////////////////////////////////////////////////////////////////////////////////////////////////
/// Draw points

// gl setup
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);

  // draw 3D will draw rays for omni pixels, and 3D points for stereo
  glLineWidth(0.3);
  glColor3f(1.0f,0.0f,0.0f);
  glBegin(GL_LINES);
  glVertex3f(camera[0], camera[1], camera[2]);
  glVertex3f(landmark[0], landmark[1], landmark[2]);
  glEnd();

  // cleanup opengl settings
  glEnable(GL_LIGHTING);
  glPopAttrib();
  return this;
}
