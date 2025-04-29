# Implementation Details

## Overview

```
                  Measurements

                    /  |  \
                   |   |   |
                   V   V   V
  +-------------------------------------------------------+
  |                                                       |
  |            [ Candidate Query ]                        |
  |                                                       |
  |               /    |    \                             |
  |  Candidate   |     |     |  Candidate                 | <-- Map Matcher
  |  cluster     V     V     V  cluster                   |
  |                                                       |
  |            [  Map Matching  ] <--- [ Viterbi Search ] |
  |                                \__ [ Routing ]        |
  |              /     |      \                           |
  +-------------|------|-------|--------------------------+
                V      V       V

                 Match  Results
```

## Candidate Query
`valhalla/meili/candidate_query.h`

Given a position and a radius, this component finds all underlying
road segments lying within this radius around the position. For every
incoming measurement we need to perform such a query to find a cluster
of candidates around it, which will be used as input to the map
matching component.

The spatial query algorithm used in Meili is simple and
efficient. Before the query, we spatially divide a road network
(i.e. a graph tile) into a grid of 500x500 (`grid.size`) squares. Then
we precompute which road segments each square intersects, and add them
to the square. The query is simply to retrieve all road segments from
the squares that the radius range covers. See [this slide]
(http://www.cs.princeton.edu/courses/archive/fall05/cos226/lectures/geosearch.pdf)
at page 7 for details.

## Map Matching
`valhalla/meili/map_matching.h`

The `MapMatching` class is the core component that implements the
HMM-based map matching algorithm. It takes a sequence of candidate
clusters as input, and picks one candidate from each cluster to form
the most likely sequence of candidates (Viterbi path). It delegates
the actual search task to the Viterbi Search module, but it defines
how to quantify the likelihood. Concretely speaking it inherits from
the `ViterbiSearch` class and implements its virtual costing functions
namely `ViterbiSearch::TransitionCost` and
`ViterbiSearch::EmissionCost`.

### State
`valhalla/meili/map_matching.h`

When feeding a cluster of candidates into the component, an *unique*
ID and an *identical* time will be attached to each candidate. The ID
identifies a state, whereas the time tells from which cluster a
candidate comes. Internally we name the wrapped candidate as a
*state*.

### Viterbi Search
`valhalla/meili/viterbi_search.h`

This module focus on finding the most likely sequence (Viterbi path)
in a [trellis graph](https://en.wikipedia.org/wiki/Trellis_(graph)) in
context of HMM model. It provides an uniform interface
`IViterbiSearch` and two implementations: `NaiveViterbiSearch`
implements the
[naive Viterbi Algorithm](https://en.wikipedia.org/wiki/Viterbi_algorithm)
and `ViterbiSearch` implements the lazy Dijkstra-based Viterbi
Algorithm. Both implementations are able to find the optimal
path. `NaiveViterbiSearch` is able to work with both maximum and
minimum objectives, whereas `ViterbiSearch` only works with minimum
objectives as it's Dijkstra-based.

We derive `MapMatching` from `ViterbiSearch` for it has better
performance in theory. You can develop your own map matching algorithm
to work with other road network sources (e.g. pgRouting) as
`MapMatching` does: inherit from either implementation (depending on
your objectives) and implement `IViterbiSearch::TransitionCost` and
`IViterbiSearch::EmissionCost`.

The details about these algorithms are described
[here](algorithms.md).

### Routing
`valhalla/meili/routing.h`

This module focuses on finding the shortest path (SP) between two
candidates in the road network. The path distance is required in the
transition cost calculation (`MapMatching::TransitionCost`). The path
will be the inferred path between their respective measurements if
both candidates are picked as part of the most likely sequence.

The SP algorithm is based on AStar, and it routes from single origin
to multiple destinations. AStar fits here because the destination set
is a cluster of candidates around their measurement (provided by the
candidate query above). So the algorithm can estimate the heuristic
cost by targeting at the measurement's position.

The SP algorithm doesn't construct the paths for you. Instead it gives
back the search tree (i.e. `LabelSet`) directly. Then we store the
search tree in the origin state for path construction later.

Turn cost between road segments along the path is aggregated during
the path finding. This cost contributes as an independent part of the
transition cost (`MapMatching::TransitionCost`) to penalize paths with
turns.

Unlike path algorithms in Thor, the SP algorithm scans nodes instead
of edges, so turn restriction is not considered here.

### Viterbi Search VS. Routing

It is worth mentioning that they share some similarities but also some
differences. Both are finding optimal paths but different objectives
(most likely sequence vs. shortest distance path). Both are based on
the Dijkstra algorithm (thank you Dijkstra!) but different graphical
models (trellis graph vs. road network).

## Map Matcher
`valhalla/meili/map_matcher.h`

A map matcher is a facade component that connects the candidate query
component and map matching component, and provides simple interfaces
for use. As shown in the overview, you can think of it as a black box
that takes measurements as input and gives back match results. In
addition, it does some internal filtering work before measurements
feeding into the map matching component.

### Interpolation
`valhalla/meili/map_matcher.h`

One thing, which is not shown in the overview, is that not all
incoming measurements are sent to the map matching component. If some
successive measurements are too spatially close to each other, then
only the first measurement will be sent; the rest of measurements will
be interpolated into the match route.

For example, assume the numbers below represent a sequence of
measurements (in order by numbers) along a straight road, and each
space is one meter long. If the `interpolation_distance` is set to 10
meters, we will only send measurements `1*`, `4*` and `7*` because
they are farther apart than 10 meters; measurements `2` and `3` will be
interpolated into the route from `1*` to `4*`; measurement `5` will be
interpolated into the route from `4*` to `7*`, and so on.

```
1* 2  3                    4*  5                  8 7* 9 10
```

The first rationale of this design is that for high-density traces it
can reduce the number of measurements involved in map
matching. Secondly if two successive measurements are too close, the
later one is possible to be found at the upstream of the earlier one
due to noise error. This error can result in wrong path inference in
some modes such as `auto`, `bicycle` where U-turns are forbidden. For
example, the true location of `8` should be at the downstream (right
side) of `7*`, but the noise can shift it to upstream (left side). In
`auto` mode, this slight shift can result in either wrong path or no
path found from `7*` to `8`. If we interpolate measurement `8` instead
of map matching it, this issue can be avoided.

### Match Result
`valhalla/meili/match_result.h`

Each measurement corresponds to a match result. The match result tells
you on which road segment the measurement gets matched or
interpolated, the match position, and the distance to the position,
etc. The corresponding state ID is attached to the result if the
measurement gets matched. Since we have stored route search trees to
states, you can find the state with this ID and reconstruct the
route with the helpers from `valhalla/meili/match_route.h`.

## Map Matcher Factory
`valhalla/meili/map_matcher_factory.h`

The map matcher factory can facilitate map matcher creation. Pass it
the Valhalla configuration and the travel mode, it reads the
parameters and creates a map matcher for this mode. The factory also
maintains a graph tile reader instance and a candidate query instance
and shares them among all its matchers. Because of the data sharing it
is cheap to create a map matcher from a factory. Note that both
factory and matcher are not thread-safe.
