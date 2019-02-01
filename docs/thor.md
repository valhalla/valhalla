# Thor

Thor serves as a routing engine backed by tiled open source routing data. Thor is a companion to Sif which it relies heavily on to determine the appropriate graph traversal. The resulting path can be used as input for creating guidance/narrative. The name Thor was chosen as an acronym standing for: Tiled Hierarchical Open Routing and was the foundational idea around which the organization Valhalla and its Norse mythology theme was formed.

The thor library is essentially a set of various data structures and alogrithms which deal with things like: A* graph traversal, edge costing, vertex costing and path construction. It also includes methods for computing time-distance matrices, optimized routing, and isochrones.

## Components ##

What follows are some notable components of thor.

### PathAlgorithm ###

  Valhalla uses a couple different algorithms to generate the route path. Thor contains a base class, called *PathAlgorithm*, defining the route path comptution interface. The following classes are derived from *PathAlgorithm*:
  - *AStar* - This is a forward direction A* algorithm which is currently used only for “trivial paths” where the origin and destination are on the same edge or adjacent, connected edges.
  - *TimeDepForward* - This is a forward direction A* algorithm meant to be used for time dependent routes where a departure time from the origin is specified. 
  - *TimeDepReverse* - This is a revers direction A* algorithm meant to be used for time dependent routes where an arrival time at the destination is specified.
  - *BidirectionalAStar* - This is a bidirectional A* algorithm used for routes that are not time-dependent and are not trivial.
  - *MultiModal* - This is a forward direction A* algorithm with transit schedule lookup included as well as logic to switch modes between pedestrian and transit. This algorithm is time-dependent due to the nature of transit schedules.

### TripPathBuilder ###

The *PathAlgorithm* methods all form a simple definition of the route path containing the directed edges comprising the path as well as the elapsed time at each edge along the path. This list is sent to a class called *TripPathBuilder* to form a more detailed representation of the trip path. *TripPathBuilder* forms details along the path that are required for guidance or narrative generation. Forming this *TripPath* requires reading attribution such as names, geometry, and other information required so that the subsequent guidance generation processing does not have to access the Valhalla tiles.

### Matrix, Isochrone, Optimized Routes ###
Thor also includes methods to compute time-distance matrices, isochrones, and optimized routes (Traveling Salesman Problem).
