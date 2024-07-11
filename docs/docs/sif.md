# Sif #

Sif provides dynamic, extensible costing for edges and transitions between edges (turn costs). Its primary use is in the [routing engine](https://github.com/valhalla/thor) when forming the best path between locations. 

Sif is essentially a set of various data structures and algorithms which deal with things like: correlating an input location to the underlying graph, partial distance along an edge and filtering edges which shouldn't be considered for correlation.
