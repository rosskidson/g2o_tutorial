// g2o example --- main.cpp
// Ross Kidson 19-1-14
//

#include <Eigen/Core>

#include "custom_types/camera_projection.h"
#include "custom_types/edge_pose_landmark_reproject.h"
#include "custom_types/vertex_landmarkxyz.h"
#include "custom_types/vertex_pose.h"
#include "custom_types/register_types.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>

// Landmark generation parameters
const int NUM_LANDMARKS = 40;
const double X_MIN = -4.0;
const double X_MAX =  4.0;
const double Y_MIN = -4.0;
const double Y_MAX =  4.0;
const double Z_MIN = 10.0;
const double Z_MAX = 15.0;

// camera parameters
const double F_X = 1000;
const double F_Y = 1000;
const double C_X = 500;
const double C_Y = 500;

// noise to be added to initial estimates
const double X_NOISE = 2.0;
const double Y_NOISE = 2.0;
const double Z_NOISE = 2.0;

// noise to be added to observations (pixels)
const double U_NOISE = 3.0;
const double V_NOISE = 3.0;

// should be done as singleton
class UniqueId
{
public:
  UniqueId():unique_id(0){}
  int getUniqueId()
  {
    return unique_id++;
  }
private:
  int unique_id;
};
static UniqueId uniqueId;

Eigen::Isometry3d isometryFromArray7D(const double* v)
{
  Eigen::Isometry3d result;

  result = Eigen::Quaterniond(v[6], v[3], v[4], v[5]).normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
  return result;
}

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void addNoise(Eigen::Vector3d & input_vec)
{
  input_vec[0] += fRand(-X_NOISE, X_NOISE);
  input_vec[1] += fRand(-Y_NOISE, Y_NOISE);
  input_vec[2] += fRand(-Z_NOISE, Z_NOISE);
}

void addNoise(Eigen::Vector2d & input_vec)
{
  input_vec[0] += fRand(-U_NOISE, U_NOISE);
  input_vec[1] += fRand(-V_NOISE, V_NOISE);
}

void generatePoses(std::vector<Eigen::Isometry3d>& pose_list)
{
  // x y z qx qy qz qw
  double pose_1 [7] = {0.00, 0.0, 0.00, 0.0,  0.0, 0.0, 1.0};
  double pose_2 [7] = {2.50, 0.0, 2.50, 0.0, -0.1, 0.0, 1.0};
  double pose_3 [7] = {5.00, 0.0, 5.00, 0.0, -0.3, 0.0, 1.0};
  double pose_4 [7] = {7.50, 0.0, 7.50, 0.0, -0.5, 0.0, 1.0};
  double pose_5 [7] = {11.00, 0.0, 9.90, 0.0, -0.7, 0.0, 1.0};
  pose_list.push_back(isometryFromArray7D(pose_1));
  pose_list.push_back(isometryFromArray7D(pose_2));
  pose_list.push_back(isometryFromArray7D(pose_3));
  pose_list.push_back(isometryFromArray7D(pose_4));
  pose_list.push_back(isometryFromArray7D(pose_5));
}

void generateLandmarks(std::vector<Eigen::Vector3d>& landmarks)
{
  landmarks.resize(NUM_LANDMARKS);
  for(int i=0; i< NUM_LANDMARKS; i++)
  {
    // create landmark
    landmarks[i][0] = fRand(X_MIN, X_MAX);
    landmarks[i][1] = fRand(Y_MIN, Y_MAX);
    landmarks[i][2] = fRand(Z_MIN, Z_MAX);
  }
}

void addPosesToGraph(std::vector<Eigen::Isometry3d>& pose_list, std::vector<int>& poseNum2Id, g2o::OptimizableGraph* graph_ptr)
{
  for(std::vector<Eigen::Isometry3d>::const_iterator itr = pose_list.begin(); itr != pose_list.end(); itr++)
  {
    const Eigen::Isometry3d & pose = *itr;
    VertexPose * v = new VertexPose();

    // add some noise to translation component
    Eigen::Vector3d t = pose.translation();
    addNoise(t);
    Eigen::Isometry3d pose_estimate(pose);
    pose_estimate.translation() = t;

    // get id
    const int id = uniqueId.getUniqueId();
    poseNum2Id.push_back(id);

    // populate g2o vertex object and add to graph
    v->setEstimate(pose_estimate);
    v->setId(id);
    v->setFixed(false);
    graph_ptr->addVertex(v);
  }
}

void addLandmarksToGraph(std::vector<Eigen::Vector3d>& landmarks, std::vector<int>& landmarkNum2Id, g2o::OptimizableGraph* graph_ptr)
{
  for(std::vector<Eigen::Vector3d>::const_iterator itr = landmarks.begin(); itr != landmarks.end(); itr++)
  {
    const Eigen::Vector3d & landmark = *itr;
    VertexLandmarkXYZ * v = new VertexLandmarkXYZ;

    // add some noise to the estimate before adding to the graph
    Eigen::Vector3d landmark_estimate(landmark);
    addNoise(landmark_estimate);

    // get ID
    const int id = uniqueId.getUniqueId();
    landmarkNum2Id.push_back(id);

    // populate g2o vertex object and add to graph
    v->setEstimate(landmark_estimate);
    v->setId(id);
    v->setFixed(false);
    graph_ptr->addVertex(v);
  }
}

void addObservationToGraph(const Eigen::Vector2d obs, const int pose_id, const int landmark_id, g2o::OptimizableGraph * graph_ptr)
{
  EdgePoseLandmarkReproject * e = new EdgePoseLandmarkReproject;

  // retrieve vertex pointers from graph with id's
  g2o::OptimizableGraph::Vertex * pose_vertex
        = dynamic_cast<g2o::OptimizableGraph::Vertex*>
          (graph_ptr->vertices()[pose_id]);

  g2o::OptimizableGraph::Vertex * point_vertex
        = dynamic_cast<g2o::OptimizableGraph::Vertex*>
           (graph_ptr->vertices()[landmark_id]);

  // error check vertices and associate them with the edge
  assert(pose_vertex!=NULL);
  assert(pose_vertex->dimension() == 6);
  e->vertices()[0] = pose_vertex;

  assert(point_vertex!=NULL);
  assert(point_vertex->dimension() == 3);
  e->vertices()[1] = point_vertex;

  // add information matrix
  Eigen::Matrix<double, 2, 2> Lambda;
  Lambda.setIdentity();

  // set the observation and imformation matrix
  e->setMeasurement(obs);
  e->information() = Lambda;

  // this edge uses camera object parameter
  e->resizeParameters(1);
  bool param_status = e->setParameterId(0, 0);
  assert(param_status);

  // finally add the edge to the graph
  if(!graph_ptr->addEdge(e))
  {
    assert(false);
  }
}

int main(int argc, char** argv)
{
  init_g2o_types();
 ////////////////////////////////////////////////////////////////////////////////////////////////////
  // initialize graph

  g2o::SparseOptimizer graph;

 ////////////////////////////////////////////////////////////////////////////////////////////////////
  // camera projection object
  std::cout << "Creating camera model...\n";

  CameraProjection * cam_ptr = new CameraProjection(F_X, F_Y, C_X, C_Y);
  cam_ptr->setId(0);
  if (!graph.addParameter(cam_ptr))
  {
    assert(false);
  }

 ////////////////////////////////////////////////////////////////////////////////////////////////////
  // poses
  std::cout << "Generating Poses...\n";

  std::vector<Eigen::Isometry3d> pose_list;
  generatePoses(pose_list);

  std::vector<int> poseNum2Id;  //used to associate pose number to g2o vertex id
  addPosesToGraph(pose_list, poseNum2Id, &graph);

 ////////////////////////////////////////////////////////////////////////////////////////////////////
  // landmarks
  std::cout << "Generating Landmarks...\n";

  std::vector<Eigen::Vector3d> landmarks;
  generateLandmarks(landmarks);

  std::vector<int> landmarkNum2Id;  //used to associate landmark number to g2o vertex id
  addLandmarksToGraph(landmarks, landmarkNum2Id, &graph);

 ////////////////////////////////////////////////////////////////////////////////////////////////////
  //observations
  std::cout << "Generating Observations...\n";

  // iterator over poses
  for(std::vector<Eigen::Isometry3d>::const_iterator pose_itr = pose_list.begin(); pose_itr != pose_list.end(); pose_itr++)
  {
    const Eigen::Isometry3d & w_T_cam = *pose_itr;
    const int pose_id = poseNum2Id[pose_itr - pose_list.begin()];

    //iterate over landmarks
    for(std::vector<Eigen::Vector3d>::const_iterator landmark_itr = landmarks.begin(); landmark_itr != landmarks.end(); landmark_itr++)
    {
      const Eigen::Vector3d & P_w = *landmark_itr;
      const int landmark_id = landmarkNum2Id[landmark_itr - landmarks.begin()];

      //landmarks are stored in global coords, convert to local coords
      Eigen::Vector3d P_cam = w_T_cam.inverse() * P_w;

      //reproject to image coordinates
      Eigen::Vector2d observation = cam_ptr->cam_map(P_cam);

      // add noise to observation
      addNoise(observation);

      //add observation to graph
      addObservationToGraph(observation, pose_id, landmark_id, &graph);
    }
  }

 ////////////////////////////////////////////////////////////////////////////////////////////////////
  // finished populating graph, export and quit
  graph.initializeOptimization();
  std::cout << "Saving Graph to example_g2o.g2o...\n";
  graph.save("example_g2o.g2o");
  std::cout << "Finished\n";
  std::cout << "rosrun g2o_viewer g2o_viewer example_g2o.g2o\n";
}
