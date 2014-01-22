#ifndef G2O_VERTEX_LANDMARKXYZ_H_
#define G2O_VERTEX_LANDMARKXYZ_H_

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

  /**
   * \brief Vertex for a tracked point in space
   */
  class VertexLandmarkXYZ : public g2o::BaseVertex<3, Eigen::Vector3d>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexLandmarkXYZ() {}
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void setToOriginImpl() { _estimate.fill(0.); }

      virtual void oplusImpl(const double* update_) {
        Eigen::Map<const Eigen::Vector3d> update(update_);
        _estimate += update;
      }

  };

#ifdef G2O_HAVE_OPENGL
  /**
   * \brief visualize a 3D point
   */
  class VertexLandmarkXYZDrawAction: public g2o::DrawAction{
    public:
      VertexLandmarkXYZDrawAction();
      virtual g2o::HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element,
         g2o::HyperGraphElementAction::Parameters* params_);


    protected:
      g2o::FloatProperty *_pointSize;
      virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
  };
#endif

#endif
