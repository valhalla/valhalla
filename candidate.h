/* -*- mode: c++ -*- */

#include <valhalla/baldr/graphid.h>

using namespace valhalla;


class Candidate
{
 public:
  Candidate(const baldr::PathLocation& pathlocation,
            double sq_distance)
      : pathlocation_(pathlocation),
        sq_distance_(sq_distance) {
  }

  inline const baldr::PathLocation& pathlocation() const {return pathlocation_;}
  inline float sq_distance() const {return sq_distance_;}
  inline float distance() const {return std::sqrt(sq_distance_);}

 private:
  baldr::PathLocation pathlocation_;
  float sq_distance_;
};
