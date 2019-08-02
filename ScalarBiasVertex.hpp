#ifndef __SCALAR_BIAS_VERTEX_HPP__
#define __SCALAR_BIAS_VERTEX_HPP__

#include <g2o/core/base_vertex.h>
#include "VertexIDManager.hpp"

class ScalarBiasVertex : public g2o::BaseVertex<1, double>
{
public:
  ScalarBiasVertex()
  {
    setId(VertexIDManager::issueID());
  }

  virtual ~ScalarBiasVertex() {}
  
  // Read and write methods automatically handled in TimeAnchoredVertex base class
  
  virtual void setToOriginImpl()
  {
    _estimate = 0;
  }

  virtual void oplusImpl(const double* update)
  {
    _estimate += * update;
  }

  virtual bool read(std::istream& is)
  {
    is >> _estimate;
    return is.good();
  }
  
  virtual bool write(std::ostream& os) const
  {
    os << _estimate << " ";
    return os.good();
  }
};

#endif // __SCALAR_BIAS_VERTEX_HPP__
