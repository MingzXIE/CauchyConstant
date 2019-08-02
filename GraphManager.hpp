#ifndef __G2O_MANAGER_HPP__
#define __G2O_MANAGER_HPP__

/**
 * \brief Centralised management of all g2o functions.
 */

#include <iostream>

#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/parameter.h>
#include <memory>

class GraphManager
{
  public:
    enum ValidationMode
    {
      GAUGE_FREEDOM = 1,
      ALL_EDGES_HAVE_RIGHT_NUMBERS_OF_NODES = 2 * GAUGE_FREEDOM,
      FULLY_CONNECTED = 2 * ALL_EDGES_HAVE_RIGHT_NUMBERS_OF_NODES,
      INFORMATION_MATRICES_PSD = 2 * FULLY_CONNECTED,
      JACOBIAN_FULL_RANK = 2 * INFORMATION_MATRICES_PSD,
      ALL = GAUGE_FREEDOM + ALL_EDGES_HAVE_RIGHT_NUMBERS_OF_NODES
          + FULLY_CONNECTED + INFORMATION_MATRICES_PSD + JACOBIAN_FULL_RANK
    };

    enum AdditionMode
    {
      IMMEDIATE = 0, BATCH = 1
    };

    GraphManager(const std::string& name);

    virtual ~GraphManager();

    bool addVertex(g2o::OptimizableGraph::Vertex* vertex,
                   g2o::OptimizableGraph::Data* data = 0);

    bool addEdge(g2o::OptimizableGraph::Edge* edge);

    bool addParameter(g2o::Parameter* parameter);

    void computeInitialGuess();

    void optimize(int iterations = 10);

    void validate(ValidationMode validationMode = ALL);

    void setAdditionMode(AdditionMode additionMode)
    {
      // If the addition mode is changing from BATCH to IMMEDIATE,
      // commit all changes to the graph
      if ((_additionMode == BATCH) && (additionMode == IMMEDIATE))
        {
          commitChanges();
        }
      _additionMode = additionMode;
    }

    g2o::SparseOptimizer& getGraph()
    {
      return *(_graph);
    }

    const g2o::SparseOptimizer& getGraph() const
    {
      return *(_graph);
    }

    bool save(const char* fileName) const;

    bool load(const char* fileName) const;

    void showNEES();

  protected:
    // This method commits all the batched changes into the graph
    void commitChanges();

  private:
    // The name of this graph
    std::string _name;

    // The solver and associated optimiser
    g2o::SparseOptimizer* _graph;

    // Flag indicates if initialized
    bool _initialized;

    // The mode for determining how vertices and edges are entered
    // into the graph
    AdditionMode _additionMode;

    // These cache the new edges and vertices
    g2o::HyperGraph::VertexSet _verticesAdded;
    g2o::HyperGraph::EdgeSet _edgesAdded;
    std::set<g2o::Parameter*> _parametersAdded;
};

#endif // __G2O_MANAGER_HPP__
