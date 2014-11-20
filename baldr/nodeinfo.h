#ifndef __nodeinfo_h_
#define __nodeinfo_h_

#include "geosupport/geosupport.h"

/**
 * Information held for each node within the graph.
 * @author  David W. Nesbitt
 */
class NodeInfo {
 public:
  /**
   * Constructor
   */
  NodeInfo() {
    latlng_.Set(0.0f, 0.0f);
  }

  /**
   * Sets the latitude and longitude.
   * @param  ll  Lat,lng position of the node.
   */
  void SetLatLng(const PointLL& ll) {
    latlng_ = ll;
  }

  /**
   * Get the latitude, longitude of the node.
   * @return  Returns the latitude and longitude of the node.
   */
  const PointLL& LatLng() const {
    return latlng_;
  }

 protected:
   // Latitude, longitude position of the node.
   PointLL latlng_;
};

#endif
