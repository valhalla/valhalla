# Sif #

Sif provides dynamic, extensible costing for edges and transitions between edges (turn costs). Its primary use is in the [routing engine](https://github.com/valhalla/thor) when forming the best path between locations. In keeping with the Norse mythological theme, the name [Sif](http://en.wikipedia.org/wiki/Sif) was chosen since Sif is a companion to Thor.

Sif is essentially a set of various data structures and alogrithms which deal with things like: correlating an input location to the underlying graph, partial distance along an edge and filtering edges which shouldn't be considered for correlation.

## Components ##

What follows are some notable components of sif.

### Dynamic Costing ###

TODO:
