
#ifndef G2O_VERTEX_POSE_
#define G2O_VERTEX_POSE_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

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

      Eigen::Matrix3d fromCompactQuat(const Eigen::Vector3d& v) {
        double w = 1-v.squaredNorm();
        if (w<0)
          return Eigen::Matrix3d::Identity();
        else
          w=sqrt(w);
        return Eigen::Quaterniond(w, v[0], v[1], v[2]).toRotationMatrix();
      }

      Eigen::Isometry3d fromVectorQuat(const Eigen::Matrix<double, 6, 1>& v){
        Eigen::Isometry3d t;
        t = fromCompactQuat(v.block<3,1>(3,0));
        t.translation() = v.block<3,1>(0,0);
        return t;
      }

      /**
       * compute a fast approximation for the nearest orthogonal rotation matrix.
       * The function computes the residual E = RR^T - I which is then used as follows:
       * R := R - 1/2 R E
       */
      template <typename Derived>
      void approximateNearestOrthogonalMatrix(const Eigen::MatrixBase<Derived>& R)
      {
        Eigen::Matrix3d E = R.transpose() * R;
        E.diagonal().array() -= 1;
        const_cast<Eigen::MatrixBase<Derived>&>(R) -= 0.5 * R * E;
      }

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
