
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include "camera_projection.h"
#include "edge_pose_landmark_reproject.h"
#include "edge_pose_pose.h"
#include "vertex_landmarkxyz.h"
#include "vertex_pose.h"

#include <iostream>


bool init_g2o_types()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("CAMERA_PROJECTION", new g2o::HyperGraphElementCreator<CameraProjection>);
  factory->registerType("EDGE_POSE_LANDMARK", new g2o::HyperGraphElementCreator<EdgePoseLandmarkReproject>);
  factory->registerType("EDGE_POSE_POSE", new g2o::HyperGraphElementCreator<EdgePosePose>);
  factory->registerType("VERTEX_LANDMARK_XYZ", new g2o::HyperGraphElementCreator<VertexLandmarkXYZ>);
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);

  return true;
}

G2O_REGISTER_ACTION(EdgePoseLandmarkReprojectDrawAction);
G2O_REGISTER_ACTION(EdgePosePoseDrawAction);
G2O_REGISTER_ACTION(VertexLandmarkXYZDrawAction);
G2O_REGISTER_ACTION(VertexPoseDrawAction);
