#include "GraphManager.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/core/factory.h>
#include <g2o/core/hyper_dijkstra.h>
#include <g2o/stuff/color_macros.h>
#include <Eigen/Eigenvalues>

using namespace ::g2o;
using namespace ::Eigen;
using namespace ::std;

GraphManager::GraphManager(const string& name) :
    _name(name)
{
  // Set the addition model to immediate
  _additionMode = IMMEDIATE;

  // Flag not initialised
  _initialized = false;

  // Set up the optimisation algorithm
  OptimizationAlgorithm* optimizationAlgorithm =
      new OptimizationAlgorithmLevenberg(
          g2o::make_unique<BlockSolverX>(
              g2o::make_unique<LinearSolverCholmod<BlockSolverX::PoseMatrixType>>()));

  // Create the graph and configure it
  _graph = new SparseOptimizer();
  _graph->setVerbose(true);
  _graph->setAlgorithm(optimizationAlgorithm);
}

bool GraphManager::addVertex(OptimizableGraph::Vertex* vertex,
                             OptimizableGraph::Data* userData)
{
  if (_additionMode == BATCH)
    {
      if (userData != 0)
        {
          vertex->setUserData(userData);
        }
      _verticesAdded.insert(vertex);
      return true;
    }
  else
    {
      _initialized = false;
      return _graph->addVertex(vertex, userData);
    }
}

bool GraphManager::addEdge(g2o::OptimizableGraph::Edge* edge)
{
  if (_additionMode == BATCH)
    {
      _edgesAdded.insert(edge);
      return true;
    }
  else
    {
      _initialized = false;
      return _graph->addEdge(edge);
    }
}

bool GraphManager::addParameter(Parameter* parameter)
{
  if (_additionMode == BATCH)
    {
      _parametersAdded.insert(parameter);
      return true;
    }
  else
    {
      _initialized = false;
      return _graph->addParameter(parameter);
    }
}

void GraphManager::commitChanges()
{
  if (_additionMode == IMMEDIATE)
    {
      if (_graph->initializeOptimization() == false)
        {
          cerr << __PRETTY_FUNCTION__
              << ": Initialisation of the optimiser failed" << endl;
        }
      return;
    }

  // Insert the parameters
  for (auto parameter : _parametersAdded)
    {
      _graph->addParameter(parameter);
    }

  if (_graph->updateInitialization(_verticesAdded, _edgesAdded) == false)
    {
      cerr << __PRETTY_FUNCTION__
          << ": Update initialisation of the optimiser failed" << endl;
    }
  _parametersAdded.clear();
  _verticesAdded.clear();
  _edgesAdded.clear();
  _initialized = true;
}

void GraphManager::validate(ValidationMode validationMode)
{
  // Return if nothing to check
  if (_graph->vertices().size() == 0)
    {
      cerr << CL_RED(__PRETTY_FUNCTION__ << ": The graph contains no nodes")
          << endl;
      return;
    }

  // Return if nothing to check
  if (_graph->edges().size() == 0)
    {
      cerr << CL_RED(__PRETTY_FUNCTION__ << ": The graph contains no edges")
          << endl;
      return;
    }

  // Check if there is a gauge freedom
  if (validationMode & GAUGE_FREEDOM)
    {
      bool gaugeFreedom = _graph->gaugeFreedom();
      OptimizableGraph::Vertex* gauge = _graph->findGauge();
      if (gaugeFreedom == true)
        {
          if (gauge == 0)
            {
              cerr
                  <<CL_RED(__PRETTY_FUNCTION__ << ": The graph has a gauge freedom")
                  << endl;
            }
          else
            {
              cout
                  <<CL_GREEN(__PRETTY_FUNCTION__ << ": The graph is fixed by node " << gauge->id())
                  << endl;
            }
        }
      else
        {
          cout
              <<CL_GREEN(__PRETTY_FUNCTION__ << ": The graph is fixed by the priors")
              << endl;
        }
    }

  if (validationMode & ALL_EDGES_HAVE_RIGHT_NUMBERS_OF_NODES)
    {
    }

  // Check that the graph is fully connected and that all vertices can
  // be reached
  if (validationMode & FULLY_CONNECTED)
    {
      HyperDijkstra d(_graph);
      UniformCostFunction f;
      HyperGraph::VertexIDMap& vertices = _graph->vertices();
      HyperGraph::VertexIDMap::iterator it = vertices.begin();
      cout << it->second << endl;

      d.shortestPaths(it->second, &f);

      if (d.visited().size() == _graph->vertices().size())
        {
          if (d.visited().size() == 1)
            {
              cout << __PRETTY_FUNCTION__ << ": The single vertex is reachable"
                  << endl;
            }
          else
            {
              cout << __PRETTY_FUNCTION__ << ": All " << d.visited().size()
                  << " vertices are reachable" << endl;
            }
        }
      else
        {
          cerr << __PRETTY_FUNCTION__ << ": Could only visit "
              << d.visited().size() << " out of " << _graph->vertices().size()
              << " vertices" << endl;
        }
    }

  // Check that all the information matrices are PSD; note that some
  // matrices might be 0 dimensional if, for example, they are a
  // RootEdge and contain no information.
  if (validationMode & INFORMATION_MATRICES_PSD)
    {
      bool allEdgeOk = true;
      SelfAdjointEigenSolver < MatrixXd > eigenSolver;
      for (OptimizableGraph::EdgeSet::const_iterator it =
          _graph->edges().begin(); it != _graph->edges().end(); ++it)
        {
          OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
          Eigen::MatrixXd::MapType information(e->informationData(),
                                               e->dimension(), e->dimension());
          // test on symmetry
          bool isSymmetric = true;  //information.transpose() == information;
          bool okay = isSymmetric;
          if (isSymmetric)
            {
              // compute the eigenvalues
              eigenSolver.compute(information, Eigen::EigenvaluesOnly);
              bool isSPD = eigenSolver.eigenvalues()(0) > 0.;
              okay = okay && isSPD;
            }
          allEdgeOk = allEdgeOk && okay;
          if (!okay)
            {
              if (true)
                {
                  if (!isSymmetric)
                    {
                      cerr
                          << "Information Matrix for an edge is not symmetric:";
                    }
                  else
                    cerr << "Information Matrix for an edge is not SPD:";
                  for (size_t i = 0; i < e->vertices().size(); ++i)
                    cerr << " " << e->vertex(i)->id();
                  if (isSymmetric)
                    cerr << "\teigenvalues: "
                        << eigenSolver.eigenvalues().transpose();
                  cerr << endl;
                  cerr << "information=\n" << information << endl;
                }
            }
        }
    }

  // Check that all the Jacobians are full rank
  if (validationMode & JACOBIAN_FULL_RANK)
    {
    }
}

void GraphManager::computeInitialGuess()
{
  if (_initialized == false)
    {
      _graph->initializeOptimization();
      _initialized = true;
    }
  _graph->computeInitialGuess();
}

void GraphManager::optimize(int iterations)
{
  if (_initialized == false)
    {
      _graph->initializeOptimization();
      _initialized = true;
    }
  _graph->optimize(iterations, false);
}

GraphManager::~GraphManager()
{
  delete _graph;
}

void GraphManager::showNEES()
{
  const OptimizableGraph::EdgeContainer& edges = _graph->activeEdges();

  // Nothing to do if the graph contains no active edges
  if (edges.empty() == true)
    {
      return;
    }

  stringstream output;

  for (OptimizableGraph::EdgeContainer::const_iterator it = edges.begin();
      it != edges.end(); it++)
    {
      OptimizableGraph::Edge* e = *it;
      output << "\tEdge\t" << e->id() << "\t" << typeid(*e).name() << "\t"
          << e->chi2() << "\n";
      //s_log << _currentTime << " " <<  e->chi2() << " " << e->dimension() << "\n";
    }
  cout << "chi2 values are\n" << output.str() << endl;
}

bool GraphManager::save(const char* fileName) const
{
  ofstream outputFile(fileName);
  outputFile.precision(15);
  cout << "saving graph with " << _graph->vertices().size() << " vertices and "
      << _graph->edges().size() << " edges" << endl;
  return _graph->save(outputFile);
}

bool GraphManager::load(const char* fileName) const
{
  ifstream inputFile(fileName);
  return _graph->load(inputFile);
}
