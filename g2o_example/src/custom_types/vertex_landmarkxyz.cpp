#include "vertex_landmarkxyz.h"
#include <stdio.h>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <typeinfo>

  bool VertexLandmarkXYZ::read(std::istream& is) {
    Eigen::Vector3d lv;
    for (int i=0; i<3; i++)
      is >> lv[i];
    setEstimate(lv);
    return true;
  }

  bool VertexLandmarkXYZ::write(std::ostream& os) const {
    Eigen::Vector3d lv=estimate();
    for (int i=0; i<3; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }


#ifdef G2O_HAVE_OPENGL
  VertexLandmarkXYZDrawAction::VertexLandmarkXYZDrawAction(): g2o::DrawAction(typeid(VertexLandmarkXYZ).name()){
  }

  bool VertexLandmarkXYZDrawAction::refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_){
    if (! g2o::DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _pointSize = _previousParams->makeProperty<g2o::FloatProperty>(_typeName + "::POINT_SIZE", 1.);
    } else {
      _pointSize = 0;
    }
    return true;
  }


  g2o::HyperGraphElementAction* VertexLandmarkXYZDrawAction::operator()(g2o::HyperGraph::HyperGraphElement* element,
                     g2o::HyperGraphElementAction::Parameters* params ){

    if (typeid(*element).name()!=_typeName)
      return 0;
    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;
    VertexLandmarkXYZ* that = static_cast<VertexLandmarkXYZ*>(element);
    

    glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
    glDisable(GL_LIGHTING);
    glColor3f(0.8f,0.5f,0.3f);
    if (_pointSize) {
      glPointSize(_pointSize->value());
    }
    glBegin(GL_POINTS);
    glVertex3f((float)that->estimate()(0),(float)that->estimate()(1),(float)that->estimate()(2));
    glEnd();
    glPopAttrib();
    return this;
  }
#endif




