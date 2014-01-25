// g2o example --- main.cpp
// Ross Kidson 19-1-14
//

#include <Eigen/Core>

#include "custom_types/vertex_pose.h"
#include "custom_types/edge_pose_pose.h"
#include "custom_types/register_types.h"

#include "custom_types/camera_projection.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>

// Landmark generation parameters
const int NUM_POSES = 20;
const int NUM_CONNECTIONS_PER_VERTEX = 4;

// noise to be added to initial estimates
const double POSE_NOISE = 10.0;

// noise to be added to relative pose edge measurement
const double EDGE_NOISE = 0.05;

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

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void addNoiseToTranslation(Eigen::Isometry3d & trafo, const double noise)
{
  trafo.translation()[0] += fRand(-noise, noise);
  trafo.translation()[1] += fRand(-noise, noise);
  trafo.translation()[2] += fRand(-noise, noise);
}

Eigen::Isometry3d isometryFromArray7D(const double* v)
{
  Eigen::Isometry3d result;

  result = Eigen::Quaterniond(v[6], v[3], v[4], v[5]).normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
  return result;
}

void generatePoses(std::vector<Eigen::Isometry3d>& pose_list)
{
  const double step_arr [7] = {0.0, 0.0, 10.0, 0.0, -0.05, 0.0, 1.0};
  const double initial_arr [7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  Eigen::Isometry3d step = isometryFromArray7D(step_arr);
  pose_list.front() = isometryFromArray7D(initial_arr);
  for(std::vector<Eigen::Isometry3d>::iterator itr = pose_list.begin() + 1; itr != pose_list.end(); itr++)
  {
    Eigen::Isometry3d & cur_trafo = *itr;
    Eigen::Isometry3d & prev_trafo = *(itr-1);
    cur_trafo = prev_trafo * step;
  }
}

void addPosesToGraph(std::vector<Eigen::Isometry3d>& pose_list, std::vector<int>& poseNum2Id, g2o::OptimizableGraph* graph_ptr)
{
  for(std::vector<Eigen::Isometry3d>::const_iterator itr = pose_list.begin(); itr != pose_list.end(); itr++)
  {
    const Eigen::Isometry3d & pose = *itr;
    VertexPose * v = new VertexPose();

    // add some noise to translation component
    Eigen::Isometry3d pose_estimate(pose);
    addNoiseToTranslation(pose_estimate, POSE_NOISE);

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

void addEdge(const Eigen::Isometry3d & a_T_b, const int id_a, const int id_b,
             g2o::OptimizableGraph* graph_ptr)
{
  EdgePosePose * e = new EdgePosePose;

  // retrieve vertex pointers from graph with id's
  g2o::OptimizableGraph::Vertex * pose_a_vertex
        = dynamic_cast<g2o::OptimizableGraph::Vertex*>
          (graph_ptr->vertices()[id_a]);

  g2o::OptimizableGraph::Vertex * pose_b_vertex
        = dynamic_cast<g2o::OptimizableGraph::Vertex*>
           (graph_ptr->vertices()[id_b]);

  // error check vertices and associate them with the edge
  assert(pose_a_vertex!=NULL);
  assert(pose_a_vertex->dimension() == 6);
  e->vertices()[0] = pose_a_vertex;

  assert(pose_b_vertex!=NULL);
  assert(pose_b_vertex->dimension() == 6);
  e->vertices()[1] = pose_b_vertex;

  // add information matrix
  Eigen::Matrix<double, 6, 6> Lambda;
  Lambda.setIdentity();

  // set the observation and imformation matrix
  e->setMeasurement(a_T_b);
  e->information() = Lambda;

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
  // poses
  std::cout << "Generating Poses...\n";

  std::vector<Eigen::Isometry3d> pose_list(NUM_POSES);
  generatePoses(pose_list);

  std::vector<int> poseNum2Id;  //used to associate pose number to g2o vertex id
  addPosesToGraph(pose_list, poseNum2Id, &graph);

 ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Edges
  std::cout << "Generating edges...\n";

  // iterator over poses
  for(std::vector<Eigen::Isometry3d>::const_iterator itr = pose_list.begin(); itr != pose_list.end(); itr++)
  {
    const Eigen::Isometry3d & w_T_a = *itr;
    for(int i = 1; i <= NUM_CONNECTIONS_PER_VERTEX; i++)
    {
      std::vector<Eigen::Isometry3d>::const_iterator itr_other = itr + i;
      if(itr_other < pose_list.end())
      {
        const int pose_a_id = poseNum2Id[itr - pose_list.begin()];
        const int pose_b_id = poseNum2Id[itr_other - pose_list.begin()];
        const Eigen::Isometry3d & w_T_b = *itr_other;

        Eigen::Isometry3d a_T_b = w_T_a.inverse() * w_T_b;
        addNoiseToTranslation(a_T_b, EDGE_NOISE);
        addEdge(a_T_b, pose_a_id, pose_b_id, &graph);
      }
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
