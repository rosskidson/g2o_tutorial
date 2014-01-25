/*
 * edge_pose_pose.h
 *
 *  Created on: Jan 19, 2013
 *      Author: ross kidson
 */


#ifndef EDGE_POSE_POSE
#define EDGE_POSE_POSE

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/hyper_graph_action.h>

#include "vertex_pose.h"

//Template arguments: 6 Dimensions, store edge as Eigen::Isometry, edge is between VertexPose and VertexPose
class EdgePosePose : public  g2o::BaseBinaryEdge<6, Eigen::Isometry3d, VertexPose, VertexPose>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgePosePose()
  { }

  virtual bool
  read                       (std::istream& is);
  virtual bool
  write                      (std::ostream& os) const;
  void
  computeError               ();

};

/**
 * \brief visualize the 3D pose vertex
 */
class EdgePosePoseDrawAction: public g2o::DrawAction{
  public:
    EdgePosePoseDrawAction();
    virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element, g2o::HyperGraphElementAction::Parameters* params_);
    g2o::HyperGraphElementAction* _cacheDrawActions;
  protected:
    virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
};


#endif // EDGE_POSE_POSE
