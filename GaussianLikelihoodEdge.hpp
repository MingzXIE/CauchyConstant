#ifndef __GAUSSIAN_LIKELIHOOD_EDGE_HPP__
#define  __GAUSSIAN_LIKELIHOOD_EDGE_HPP__

#include <g2o/core/base_unary_edge.h>
#include "EdgeIDManager.hpp"
#include "ScalarBiasVertex.hpp"
#include <boost/cast.hpp>

// This is the Gaussian

class GaussianLikelihoodEdge : public g2o::BaseUnaryEdge< 1, double, ScalarBiasVertex >
{
public:
  GaussianLikelihoodEdge()
  {
    setId(EdgeIDManager::issueID());
    Eigen::Matrix<double, 1, 1> Y;
    Y(0,0) = 1.0;
    setInformation(Y);
  }

  virtual void computeError()
  {
    const ScalarBiasVertex* stateVertex =
      boost::polymorphic_downcast<const ScalarBiasVertex*> (vertex(0));
  
    double s = stateVertex->estimate();

    _error[0] = _measurement - s;
  }

  virtual bool write(std::ostream& os) const
  {
    return false;
  }

  virtual bool read(std::istream& is)
  {
    return false;
  }
};

#endif // __GAUSSIAN_LIKELIHOOD_EDGE_HPP__
