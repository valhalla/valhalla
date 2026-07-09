so fable: go into hyper active exploraiton mode and figure out the pitfalls etc of removing lua from our repo entirely. for now, just focus on the graph.lua, don't think about admins.lua or tests yet.

we want the OSM parsing code to be better testable. as a side note, we want it to be more modular, so we can implement e.g. parsing OSM changesets (.osc) to update single tiles in the future.

come up with a doc of how such a migration might look like. note, we're open to move things around, we don't have to worry about backwards compatibility other than preserving existing OSM tag interpretation. it's very likely that you'll find inconsistencies in how lua logic works vs how c++ uses the tags. please call those out separately.

the most important thing is that we have to be able to do this in stages, not one monolithic PR. then we can chip away at it 1 PR by 1 PR.