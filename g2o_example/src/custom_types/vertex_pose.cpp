
#include "vertex_pose.h"
#include "draw_functions.hpp"
#include "g2o/core/factory.h"
#include "g2o/stuff/opengl_wrapper.h"

#include <iostream>
#include "g2o/core/cache.h"

const static double DEFAULT_COORDINATE_SIZE = 1.5;

  VertexPose::VertexPose() :
    g2o::BaseVertex<6, Eigen::Isometry3d>(),
    _numOplusCalls(0)
  {
    setToOriginImpl();
    updateCache();
  }

  bool VertexPose::read(std::istream& is)
  {
    std::vector<double> est(7);
    for (int i=0; i<7; i++)
      is  >> est[i];

    Eigen::Isometry3d t;
    t= Eigen::Quaterniond(est[6], est[3], est[4], est[5]).toRotationMatrix();
    t.translation() = Eigen::Vector3d(est[0], est[1], est[2]);
    setEstimate(t);
    return true;
  }

  bool VertexPose::write(std::ostream& os) const
  {
    Eigen::Quaterniond q(_estimate.matrix().topLeftCorner<3,3>()); // extract rotation part
    q.normalize();
    std::vector<double> est(7);
    est[3] = q.x();
    est[4] = q.y();
    est[5] = q.z();
    est[6] = q.w();
    est[0] = _estimate.translation()[0];
    est[1] = _estimate.translation()[1];
    est[2] = _estimate.translation()[2];

    for (int i=0; i<7; i++)
      os << est[i] << " ";
    return os.good();
  }

#ifdef G2O_HAVE_OPENGL
  VertexPoseDrawAction::VertexPoseDrawAction(): g2o::DrawAction(typeid(VertexPose).name()){
    _cacheDrawActions = 0;
  }

  bool VertexPoseDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _coordinate_size = _previousParams->makeProperty<g2o::FloatProperty>(_typeName + "::COORDINATE_FRAME_SIZE", DEFAULT_COORDINATE_SIZE);
    } else {
      _coordinate_size = 0;
    }
    return true;
  }

  g2o::HyperGraphElementAction* VertexPoseDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element,
                 HyperGraphElementAction::Parameters* params_){

    if (typeid(*element).name()!=_typeName)
      return 0;
    if (! _cacheDrawActions){
      _cacheDrawActions = g2o::HyperGraphActionLibrary::instance()->actionByName("draw");
    }

    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;

    VertexPose* that = static_cast<VertexPose*>(element);

    glColor3f(0.5f,1.0f,1.0f);
    glPushMatrix();
    glMultMatrixd(that->estimate().matrix().data());

    // check param is not null pointer
    float coordinate_size = _coordinate_size == 0 ? DEFAULT_COORDINATE_SIZE : _coordinate_size->value();

    //make the first frame bigger
    if(that->id() == 0)
      coordinate_size *= 3;

    drawCoordinateSys(coordinate_size);

    g2o::CacheContainer* caches=that->cacheContainer();
    if (caches){
      for (g2o::CacheContainer::iterator it=caches->begin(); it!=caches->end(); it++){
        g2o::Cache* c = it->second;
        (*_cacheDrawActions)(c, params_);
      }
    }
    g2o::OptimizableGraph::Data* d=that->userData();
    while (d && _cacheDrawActions ){
      (*_cacheDrawActions)(d, params_);
      d=d->next();
    }
    glPopMatrix();
    return this;
  }

#endif
