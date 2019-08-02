// This runs a quick test of the non-Gaussian likelihood stuff from
// Rosen et al.

// #include <base/GraphManager.hpp>
#include "GraphManager.hpp"
#include <random>
#include <fstream>
#include <iostream>
#include "ScalarBiasVertex.hpp"
#include "GaussianLikelihoodEdge.hpp"
#include "CauchyLikelihoodEdge.hpp"

using namespace ::std;

// Quick serialisation test
#include <g2o/core/factory.h>
G2O_REGISTER_TYPE_GROUP(cauchy_example);
G2O_REGISTER_TYPE(SCALAR_BIAS_VERTEX, ScalarBiasVertex);


int main(int argc, const char* argv[])
{
  // Set up the two cases: the Gaussian graph using the standard
  // Gaussian likelihood, and the Cauchy graph using the Cauchy
  // distribution.
  GraphManager gaussianGraph("gaussian");
  ScalarBiasVertex* gsbv = new ScalarBiasVertex();
  gsbv->setToOrigin();
  gaussianGraph.addVertex(gsbv);

  GraphManager cauchyGraph("cauchy");
  ScalarBiasVertex* csbv = new ScalarBiasVertex();
  csbv->setToOrigin();
  cauchyGraph.addVertex(csbv);


  double a = 1;
  double b = 5;
  size_t numberOfSamples = 100;
  ofstream gaussianSolution("gaussianSolution.txt");
  ofstream cauchySolution("cauchySolution.txt");

  double sample;
  string line;
  ifstream sampleFile ("samples.txt");
  if (sampleFile.is_open())
  {
    while(getline(sampleFile,line)){


      sample = atof(line.c_str());
      // Construct an edge and insert it into the graphs
      GaussianLikelihoodEdge* gle = new GaussianLikelihoodEdge();
      gle->setMeasurement(sample);
      gle->setVertex(0, gsbv);
      gaussianGraph.addEdge(gle);
      gaussianGraph.optimize();
      gaussianSolution << gsbv->estimate() << "\n";

      // Construct an edge and insert it into the graphs
      CauchyLikelihoodEdge* cle = new CauchyLikelihoodEdge(a, b);
      cle->setMeasurement(sample);
      cle->setVertex(0, csbv);
      cauchyGraph.addEdge(cle);
      cauchyGraph.optimize();
      cauchySolution << csbv->estimate() << "\n";

    }
    sampleFile.close();
    cout<<"complete reading data"<<"\n";
  }else{
    cout<<"Unable to open file";
  }

}
