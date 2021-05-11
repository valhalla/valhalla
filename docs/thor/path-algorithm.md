## Thor - Determining the Best Path

Two core components of the Valhalla open source routing engine are **Thor** and **Sif**. These 2 companions (in Norse mythology Thor and Sif are husband and wife) form the basis of Valhalla's path generation algorithm. Thor contains the path computation algorithms and traverses the routing tiles, while Sif performs **costing** that is central to forming the best path. Rather than baking costs into the routing graph data, Valhalla uses dynamic, run-time costing to generate costs based on a rich set of attributes stored in the routing graph tiles. This allows run-time generation of different types of routes (or routes with different characteristics) simply by using different costing methods and options within those methods.

#### Path Algorithm Introduction

Routing from one location to another is solved by a class of algorithms known as **shortest path algorithms**. This is somewhat misleading, as often one is interested in a route that is shortest time or one that makes fewer turns. A better term for shortest path algorithms is **least cost algorithms** - this properly indicates that the method is minimizing cost, be it distance, time, or some other metric. Naive assignment of cost to edges of the routing graph will lead to poor routing solutions. Simple costing based solely on distance or on time (based solely on speed) can lead to poor route paths with excessive turns and stops. Considerations such as turn types, classifications of roads at intersections along the route, road surface type, elevation change, road curvature, and a host of other considerations can be important.  It is also important to note that different costing considerations are needed for bicycle routing than pedestrian routing or automobile routing. Dynamic costing is described [here](https://github.com/valhalla/valhalla/blob/master/docs/sif/dynamic-costing.md).

Valhalla uses several levels of road hierarchies to enhance performance. The lowest level hierarchy is called the local level. The local level includes all roads and paths that are routable (using various access methods). The next hierarchy level is called arterial. This level drops out all paths and residential roads. The highest level is called the highway level. This level includes just motorways, trunk roads, and primary roads - these are roads needed for long routes. By transitioning to the higher hierarchy levels as the route path moves away from the origin or destination the path finding algorithm considers less roads - improving performance. Also, shortcut edges are formed on the arterial and highway hierarchies. These edges bypass intersections that only connect to lower hierarchy edges. This allows several edges to be combined into one longer edge, which also improves performance. 

Thor uses several different algorithms to compute the least cost path. These algorithms are described below.

#### A\*

The basic algorithm provided within Thor is an A\* algorithm. This algorithm searches in one direction - from the origin towards the destination. The A\* heuristic is added to the cost to help guide the search more rapidly towards the destination. The A\* method has been superceded for most cases by the bidirectional A\* algorithm which has better performance. Also, the A\* algorithm does not work as well with transitions to upper hierarchy levels as the path approaches the destination.

#### Bidirectional A\*

The primary algorithm used for most types of routes is a bidirectional A\* method. This algorithm searches for the lowest cost path in two directions: one from the origin towards the destination and the other "backwards" from the destination towards the origin. This algorithm has better performance the the A\* algorithm since it more effectively cuts the search space. However, there are some complexities added to handle the backwards progression from the destination to the origin. Turn restrictions and transition costing is more complicated. Also, the determination of the connection point between the two searches (determination of route completion) is more complex. Another strength of the bidirectional A* method is that hierarchy transitions near the destination are simplified.

Pedestrian and bicycle routes use just the local graph hierarchy. They never transition to the arterial or highway levels and thus never use shortcut edges.

The bidirectional A\* algorithm makes use of edge markings that enter regions where no through path exists. These are areas of the routing graph that represent cul-de-sas, dead-end roads, and even larger communities where there is only one entrance. The search paths can exclude ever entering an edge that is marked as "not-though".

#### Multi-modal

Multi-modal routes use an A\* method that is enahanced to allow time-dependency and mode changes. Public transit information includes schedule information that find the next departure along directed edges between transit stops. Unique pairs of transit stops and routes create separate graph edges with a unique *line-id* to which departure schedules can be associated.

#### A* Heuristic

A simple class within Thor handles the A\* heuristic computation. At the beginning of PathAlgorithm::GetBestPath the A\* heuristic is initialized with the latitude, longitude of the destination and a costing factor to multiply distance estimates with. This factor needs to be tied to the costing model to multiply distance that will underestimate the cost to the destination, but keep close to a reasonable true cost so that performance is kept high. For example, in automobile costing the factor is based on the highest speed expected - thus any straight line distance estimate from a specific location will undersestimate the true cost on any path on real roads to get to the destination. Distance estimates are computed using a distance approximation method that computes a Euclidean distance using meters per degree of latitude and an estimate of meters per degree of longitude based on the destination latitude. This produces a close approximation of the arc distance along the surface of the earth while providing a distance measure that is locally stable (nearby locations will get consistent and close distance approximations).

#### Edge Labeling

Thor marks directed edges in the routing graph rather than nodes. This allows a node to be traversed multiple times in a route with different directed edges. This allows turn restrictions to be incorporated into the data and the path algorithm. This is demonstrated in the following example where a left turn is not allowed at an intersection. Rather the route must take a separate turn lane to the right and loop back through the intersection. The least cost path to the intersection node is to proceed straight. If the node were marked it would prevent traversing the node after using the turn lane since that path is higher cost. 

Each directed edge in the routing graph can have three states:

- **Not Visited** - An edge that is not visited has not yet been encountered within the PathAlgorithm graph traversal.
- **Temporary** - An edge that has been visited or encountered but there could still be a lower cost path to this edge. This edge will be "adjacent" or connected to an edge that is permanently labeled. Temporary edges are noted in the Adjacency List and are sorted such that they are "expanded" in order of lowest cost.
- **Permanent** - Lowest cost path to this edge has been found.

Edges that have been visited are stored in a vector with an EdgeLabel structure that contains information about the path up to this edge. In particular the predecessor edge is stored. This allows the shortest path of directed edges to be constructed by using each edges predecessor information to walk the path backwards. Additional information about the path to get to the directed edge is also kept. This information includes:

- **Edge Id** - Graph Id of the edge.
- **Opposing edge Id** - Graph Id of the opposing edge (for bidirectional A*).
- **End node** - GraphId of the end node of the edge. This allows the expansion to occur by reading the node and not having to re-read the directed edge to find its end node.
- **Cost** - Cost and elapsed time in seconds along the path to this edge.
- **Sort cost** - Cost including includes A* heuristic. 
- **Distance** - An estimate of the straight line distance to the destination.
- **Predecessor edge** - Index to the predecessor edge label information within the EdgeLabels list.

Several other pieces of information about the prior edge are also kept to avoid having to re-read an edge. In addition, several transit specific attributes are added for multi-modal routes.

#### Edge Status

An unordered map (hash map) is used to identify the state of directed edges. The map contains tile id as key and array of EdgeStatusInfo which contains 
index of the edge in the EdgeLabels vector and the current edge label state: kUnreachedOrReset, temporary or permanent.
Whenever a new tile (new edge in previously unvisited tile) is encountered a new value in the map is inserted with key as tile id and EdgeStatusInfo array of length equal to number of directed edges in the tile as value, all directed edges in the new array are initialized with kUnreachedOrReset status.

The index of edge in EdgeStatusInfo array is equal to it's id in the tile

EdgeStatus is constructed given an initial size of the edge status map. To avoid rehashing the initial size should be large enough.

- **Set** - Sets the status of a directed edge given its GraphId.
- **Update** - Updates the status of a directed edge given its GraphId.
- **Get** - Gets the status info of a directed edge given its GraphId.

#### Adjacency List

The AdjacencyList class provides a sorting order to the edge labels that are marked as temporary and are adjacent to edges that have lowest cost path found. The adjacency list uses a bucket sort implementation for performance. An "overflow" bucket is maintained to allow reduced memory use - costs outside the current bucket range get placed into the overflow bucket and are moved into the low-level buckets as needed. The adjacency list stores indexes into a list (vector) of labels where complete cost and predecessor information are stored. The adjacency list simply provides a fast sorting method. Benchmarks show a marked improvement over using an STL priority_queue, even in cases where the overflow bucket is utilized.

An AdjacencyList is contructed using a minimum cost (based on the A* heuristic distance from the origin location to the destination location), a range of costs held within the bucket sort, and a bucket size. All costs above mincost + range are stored in an "overflow" bucket. The following methods are provided in the AdjacencyList class:

- **Add** - Adds a label index to the sorted list. Adds it to the appropriate bucket given the sort cost. If the sortcost is greater than maxcost_ the label is placed in the overflow bucket. If the sortcost is < the current bucket cost then the label is placed at the front of the current bucket (this prevents underflow).
- **DecreaseCost** - The specified label index now has a smaller cost.  Reorders it in the sorted bucket list.
- **Clear** - Clear all labels from from the adjacency list. Called at the start of the path finding algorithm,
- **Remove** - Removes the lowest cost label index from the sorted list.
- **EmptyOverflow** - Empties the overflow bucket by placing the label indexes into the low level buckets. This method is private and is called from the Remove method when needed.
