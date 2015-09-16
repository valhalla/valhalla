/* -*- mode: c++ -*- */

#include <valhalla/baldr/graphid.h>

using namespace valhalla;


class Candidate
{
 public:
  Candidate(const baldr::PathLocation& pathlocation,
            float distance)
      : pathlocation_(pathlocation),
        distance_(distance) {
  }

  inline const baldr::PathLocation& pathlocation() const {return pathlocation_;}
  inline float distance() const {return distance_;}

 private:
  baldr::PathLocation pathlocation_;
  float distance_;
};
