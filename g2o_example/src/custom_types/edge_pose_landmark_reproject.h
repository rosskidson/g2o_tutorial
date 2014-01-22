/*
 * edge_pose_landmark_reproject.h
 *
 *  Created on: Jan 19, 2013
 *      Author: ross kidson
 */


#ifndef EDGE_POSE_LANDMARK_REPROJECT
#define EDGE_POSE_LANDMARK_REPROJECT

#include "camera_projection.h"
#include "vertex_pose.h"
#include "vertex_landmarkxyz.h"

const int NUM_VERTICES = 2;

class EdgePoseLandmarkReproject : public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexLandmarkXYZ>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgePoseLandmarkReproject()
  {
    resizeParameters(1);
    if(!installParameter(cam, 0, 0))
      std::cout << "problem with loading camera parameter in reprojection object\n";
    _vertices.resize(NUM_VERTICES);
    resize(NUM_VERTICES);
  }

  virtual bool
  read                       (std::istream& is);
  virtual bool
  write                      (std::ostream& os) const;
  void
  computeError               ();

  CameraProjection * cam;

};

/**
 * \brief visualize the 3D pose vertex
 */
class EdgePoseLandmarkReprojectDrawAction: public g2o::DrawAction{
  public:
    EdgePoseLandmarkReprojectDrawAction();
    virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element, g2o::HyperGraphElementAction::Parameters* params_);
    g2o::HyperGraphElementAction* _cacheDrawActions;
  protected:
    virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
};


#endif // EDGE_POSE_LANDMARK_REPROJECT
