

#include <g2o/core/base_vertex.h>
#include <GL/gl.h>


inline void drawPoint(const float size, const Eigen::Vector3d & point)
{
  glPointSize(size);
  glBegin(GL_POINTS);
  glVertex3d(point[0], point[1], point[2]);
  glEnd();
}

inline void drawTriangle(float xSize, float ySize){
  Eigen::Vector3f p[3];
  glBegin(GL_TRIANGLES);
  p[0] << 0., 0., 0.;
  p[1] << -xSize, ySize, 0.;
  p[2] << -xSize, -ySize, 0.;
  for (int i = 1; i < 2; ++i) {
    Eigen::Vector3f normal = (p[i] - p[0]).cross(p[i+1] - p[0]);
    glNormal3f(normal.x(), normal.y(), normal.z());
    glVertex3f(p[0].x(), p[0].y(), p[0].z());
    glVertex3f(p[i].x(), p[i].y(), p[i].z());
    glVertex3f(p[i+1].x(), p[i+1].y(), p[i+1].z());
  }
  glEnd();
}

inline void drawCoordinateSys(float size){
  glLineWidth(3.5);
  glDisable(GL_LIGHTING);

// X axis
  glColor3f(1.0f,0.0f,0.0f);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(size, 0, 0);
  glEnd();

// Y axis
  glColor3f(0.0f,1.0f,0.0f);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0, size, 0);
  glEnd();

// Z axis
  glColor3f(0.0f,0.0f,1.0f);
  glBegin(GL_LINES);
  glVertex3f(0.0, 0.0, 0.0);
  glVertex3f(0, 0, size);
  glEnd();


}

