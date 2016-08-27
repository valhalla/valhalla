# Implementation Details

## Overview

```
                  Measurements

                    /  |  \
                   |   |   |
                   V   V   V
  +-------------------------------------------------------+
  |                                                       |
  |           [ Candidate Query ]                         |
  |                                                       |
  |               /    |    \                             |
  |  Candidates  |     |     |  Candidates                | <-- Map Matcher
  |              V     V     V                            |
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
incoming measurement we need to perform such a query to find all
surrounding candidates, which will be used as input to the map
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

The map matching component is the core component that implements the
HMM-based map matching algorithm. It takes a list of candidate lists
as input and finds out the most likely sequence of candidates. It
delegates the actual search task to the Viterbi Search module, but it
defines how to quantify the likelihood. Concretely speaking it
inherits from the `ViterbiSearch` class and implements its virtual
costing functions namely `ViterbiSearch::TransitionCost` and
`ViterbiSearch::EmissionCost`.

### State
`valhalla/meili/map_matching.h`

When feeding a list of candidates into the component, each candidate
will be attached with an *unique* ID, and an *identical* time. The ID
is used to identify a state, whereas the time is used to identify what
time a candidate comes in, or which measurement it comes
from. Internally we name the wrapped candidate as a *state*.

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
path. `NaiveViterbiSearch` is able to work with the both maximum and
minimum objectives, whereas `ViterbiSearch` only works with minimum
objectives as it's Dijkstra-based.

We derive `MapMatching` from `ViterbiSearch` because it's faster in
theory. You can develop your own map matching algorithm to work with
other road network sources (e.g. pgRouting) by inheriting from either
implementation (depending on your objectives), and implement
`IViterbiSearch::TransitionCost` and `IViterbiSearch::EmissionCost`.

The details about these algorithms are described
[here](https://github.com/valhalla/meili/blob/master/docs/algorithms.md).

### Routing
`valhalla/meili/routing.h`

This module focus on finding the shortest path (SP) from one state to
another state in the road network. The path distance is required in
the transition cost calculation (`MapMatching::TransitionCost`). The
path is the inferred path between two measurements.

The SP algorithm only focuses on *limited distance* path finding. If
it hasn't found the destination after exploring 2000 meters
(`breakage_distance`), it will give up without saying
anything. Apparently we can't infer the path if no path is found
between two successive measurements. However they still have chances
to find the correct match road, if the former measurement can find a
path from its preceding measurement, and the later measurement can
find a path to its succeeding measurement, respectively.

The SP algorithm is based on AStar, and it routes from single origin
to multiple destinations. AStar fits here because the destination set
is a cluster of candidates within a radius around their measurement
location (provided by the candidate query above). So the algorithm can
estimate the heuristic cost by targeting at the measurement's
location.

The SP algorithm doesn't construct the paths for you. Instead it gives
back the search tree (i.e. `LabelSet`) directly. Then we store the
search tree in the origin state for path construction later.

Turn cost between road segments along the path is aggregated during
the path finding. This cost contributes as an independent part of the
transition cost (`MapMatching::TransitionCost`) to penalize U-turns in
modes that enables U-turns, such as `pedestrian`.

Unlike SP algorithms in Thor, this SP algorithm scans nodes instead of
edges, so turn restriction is not considered here.

### Viterbi Search VS. Routing

It is worth mentioning that they share some similarities but also some
differences. Both are finding optimal path but different objectives
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
they are far apart than 10 meters; measurements `2` and `3` will be
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
some modes such as `auto`, `bicycle` where U-turns is forbidden. For
example, the true location of `8` should be at the downstream (right
side) of `7*`, but the noise can shift it to upstream (left side). In
`auto` mode, this slight shift can result in either wrong path or no
path found from `7*` to `8`. If we interpolate measurement `8` instead
of map matching it, this issue can be avoided.

### Match Result
`valhalla/meili/match_result.h`

Each measurement should correspond to a match result. The match result
tells you which road segment candidate the measurement gets matched or
interpolated, the match position on the road segment, and the distance
from the measurement to the position, etc. The corresponding state ID
is also attached to the result if the measurement gets matched. Since
we have stored route search trees to states, so you can find the state
with this ID and reconstruct the route with the helpers from
`valhalla/meili/match_route.h`.

## Map Matcher Factory
`valhalla/meili/map_matcher_factory.h`

The map matcher factory can facilitate map matcher creating. Pass it
the Valhalla configuration and the travel mode, it reads the
parameters for the mode and creates the corresponding map matcher for
you. It also maintains a graph tile reader instance and a candidate
query instance, and share the references among all its matchers, so
creating a matcher is cheap. Note that both factory and matcher are
not thread-safe.
