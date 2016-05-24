/* -*- mode: c++ -*- */
#ifndef MMP_CANDIDATE_H_
#define MMP_CANDIDATE_H_

#include <cmath>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/pathlocation.h>

namespace valhalla{

namespace meili {

// A thin wrapper of PathLocation
class Candidate: public baldr::PathLocation
{
 public:
  Candidate() = delete;

  Candidate(const baldr::Location& location)
      : baldr::PathLocation(location), sq_distance_(0.f) {}

  float sq_distance() const
  { return sq_distance_; }

  Candidate& set_sq_distance(float sq_distance)
  {
    sq_distance_ = sq_distance;
    return *this;
  }

  float distance() const
  { return std::sqrt(sq_distance_); }

 private:
  float sq_distance_;
};

}

}

#endif // MMP_CANDIDATE_H_
