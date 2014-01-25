
#ifndef G2O_VERTEX_POSE_
#define G2O_VERTEX_POSE_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

#include "math_functions.hpp"

/**
 * \brief 3D pose Vertex, represented as an Isometry3d
 *
 * 3D pose vertex, represented as an Isometry3d, i.e., an affine transformation
 * which is constructed by only concatenating rotation and translation
 * matrices. Hence, no scaling or projection.  To avoid that the rotational
 * part of the Isometry3d gets numerically unstable we compute the nearest
 * orthogonal matrix after a large number of calls to the oplus method.
 * 
 * The parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
  class VertexPose : public g2o::BaseVertex<6, Eigen::Isometry3d>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      VertexPose();

      static const int orthogonalizeAfter = 1000; //< orthogonalize the rotation matrix after N updates

      virtual void setToOriginImpl() {
        _estimate = Eigen::Isometry3d::Identity();
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      /**
       * update the position of this vertex. The update is in the form
       * (x,y,z,qx,qy,qz) whereas (x,y,z) represents the translational update
       * and (qx,qy,qz) corresponds to the respective elements. The missing
       * element qw of the quaternion is recovred by
       * || (qw,qx,qy,qz) || == 1 => qw = sqrt(1 - || (qx,qy,qz) ||
       */
      virtual void oplusImpl(const double* update)
      {
        Eigen::Map<const Eigen::Matrix<double, 6, 1> > v(update);
        Eigen::Isometry3d increment = fromVectorQuat(v);
        _estimate = _estimate * increment;
        if (++_numOplusCalls > orthogonalizeAfter) {
          _numOplusCalls = 0;
          approximateNearestOrthogonalMatrix(_estimate.matrix().topLeftCorner<3,3>());
        }
      }

    protected:
      int _numOplusCalls;     ///< store how often opluse was called to trigger orthogonaliation of the rotation matrix
  };


#ifdef G2O_HAVE_OPENGL
  /**
   * \brief visualize the 3D pose vertex
   */
  class VertexPoseDrawAction: public g2o::DrawAction{
    public:
      VertexPoseDrawAction();
      virtual g2o::HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element,
                                                       g2o::HyperGraphElementAction::Parameters* params_);
      g2o::HyperGraphElementAction* _cacheDrawActions;
    protected:
      virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
      g2o::FloatProperty* _coordinate_size;
  };
#endif

#endif
