#ifndef __CAUCHY_LIKELIHOOD_EDGE_HPP__
#define  __CAUCHY_LIKELIHOOD_EDGE_HPP__

#include <g2o/core/base_unary_edge.h>
#include "EdgeIDManager.hpp"
#include "ScalarBiasVertex.hpp"
#include <boost/cast.hpp>

// This is the Gaussian

class CauchyLikelihoodEdge : public g2o::BaseUnaryEdge< 1, double, ScalarBiasVertex >
{
public:
  CauchyLikelihoodEdge(double a, double b)
  {
    setId(EdgeIDManager::issueID());
    Eigen::Matrix<double, 1, 1> Y;
    Y(0,0) = 1.0;
    setInformation(Y);
    _a = a;
    _b = b; // gamma
    _b2 = _b * _b;
    _epsilon = 1 / (M_PI * _b);
    //std::cout << "b2=" << _b2 << std::endl;
    //exit(0);
  }

  virtual void computeError()
  {
    const ScalarBiasVertex* stateVertex =
      boost::polymorphic_downcast<const ScalarBiasVertex*> (vertex(0));

    double s = stateVertex->estimate();

    double cauchyProbability = _epsilon / (1+(s -_measurement)*(s - _measurement) / (_b2));
    assert(_epsilon - cauchyProbability >= 0); // ???

    //std::cout << (s - _measurement) << std::endl;

    _error[0] = sqrt(log(_epsilon) - log(cauchyProbability));
    //std::cout << _error[0] << std::endl;
  }

  virtual bool write(std::ostream& os) const
  {
    return false;
  }

  virtual bool read(std::istream& is)
  {
    return false;
  }

private:
  double _a;
  double _b;
  double _b2;
  double _epsilon;
};

#endif // __CAUCHY_LIKELIHOOD_EDGE_HPP__
