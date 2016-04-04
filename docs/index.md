# Loki #

The loki library is essentially a set of various data structures and alogrithms which deal with things like: correlating an input location to the underlying graph, partial distance along an edge and filtering edges which shouldn't be considered for correlation.

## Components ##

What follows are some notable components of the loki library.

### Search ###

#### What's it do? ####

TODO: show a picture of correlation

The primary function of loki is to correlate the given input coordinate(s) to the underlying routing graph by searching over small portions of said graph near said input. The goal is to return information about the graph that can be used by a router to find a path in the graph from one point to another. Generally this is a set of candidate edges for each input coordinate. Additionally the actual correlated (snapped to route network geometry) coordinate is returned which can be useful for things like placing transit egress points on the route network or fun trivia stuff like dropping a point in the atlantic or a massive desert to find the closest signs of civilization to that point.

#### How's it work? ####

First, we exploit the fact that the graph is tiled and separated into levels of detail. See [Why Tiles?](../../../mjolnir/docs/why_tiles.md) for more detail. This is important because it lets us focus our search on specific partitions of the graph which are closest to the input coordinate. Also, note that loki is only interested in the most detailed level as it has all of the edges in the graph, even the lesser importance ones. You might be thinking ok so those tiles must be pretty darn small to make this reasonable (time/space complexity). Because the tiles are a regularly spaced grid, some will land on almost no routable edges and some will land on, well, Tokyo.

TODO: show a picture of the fat Tokyo tile vs the very skinny Longyearbyen

This may not seem like much of a problem at first but closer inspection reveals its very important to strike a balance between tile size and complexity concerns. The heart of the problem is that we use these tiles for several different use-cases (algorithms) and each has different needs. Performing the actual graph traversal algorithm benefits from loading larger tiles into memory so the expansion of the graph doesnt hit (load more) new tiles so frequently. At the same time you want smaller tiles so that when you need to find just a couple edges nearest to a point you don't need to consider so many edges. There are more concerns but these illustrate the point well enough.

So to settle on a size we looked at which operation dominated (in terms of complexity) with the intuition that optimizing things for that would give the most bang for the buck. That lead us to larger (for some value of large) tiles to benefit graph traversal. It did though mean that some tricks needed to be carried out on the graph correlation use-case. We're getting ahead of ourselves though. How did it work with native high detail level tile sizes? You noticed the past tense there right? Foreshadowing!

The intial implementation (lets call it v0) of this was very very simple. For a single input coordinate, loki would just open the tile that the coordinate was in, scan through all the nodes in that chunk of the routing graph keeping track of the one that was closest to the input coordinate. When it was done checking all of the nodes it would just return the edges connected to the one that was closest to the input coordinate. If there were no nodes in the tile, it would return no result. As you may have guessed though, it was really fast (sub-milisecond in Tokyo). You may also have guessed that it was really annoying. The closest node is often not even connected to the correct edge and can be pretty far away from the input coordinate in sparser tiles. This brings up another point about what a good implementation of loki should do. It should find the most plausible result not just the quickest one. This sounds obvious but of course the two things are at odds.

Next we were onto the second implementation (v1 I guess?) which entailed actually looking at the edges' shapes. Now this operation is expensive... The shapes are compressed into strings via polyline encoding, which means to look at one we need to decompress it. This is expensive, so its important to look at as few as possible. Here is where we run into a problem. We really want to look at all the edges that have shape near the input coordinate, but we have no idea which ones those are without looking at every edge's shape. So we did a few tricks. The first one was that since scanning through the nodes was free (essentially) we would do that first and if we were within a certain distance from a node we'd just return that result. This was problematic if the distance threshhold wasn't set high enough because it could snap the result to a node on an opposing lane of traffic. Tune the threshhold smaller and you make the likelihood of snapping to a node so low its not worth trying. The second trick we employed was using the length of an edge as a heuristic to determine whether an edge should or should not be considered (its shape decoded) with respect to a given input point. Basically if an edge was long enough to have reached the input point from one of its endpoints it would be considered. This weeded out a lot of short edges that werent close to the input, but it had the side effect of over emphasizing larger edges nearer to the input. In other words it was not all that fast and there were lots of problems with it: finds result in wrong tile, finds no result because its in the adjacent tile, snaps to the wrong road because of node snapping, finds the wrong road because over emphasis on longer edges (#72, #62, #52).

So we were in a bit of a terrible state... Wrong results or no results and nothing that was particularly fast (median 130ms across NYC). It was time to come back to the tile size... But honestly it was only affecting this use-case. The case in which you are using the tiles as a geospatial index to limit your search to a small geographic region. So we had an idea. Add an index to each tile at high enough resolution for it to be performant. So we hoped over to mjolnir and did it. Essentially for each tile, we cut it up into a 5x5 set of bins and for each bin we note which edges pass through it. Mjolnir is what makes our tiles, meaning we do this once at tile creation time so the index is cached with the tile (its basically appended to the end with the shape data and so forth). Another important trick we did was reference the edges in these bins in a global fashion (an edge from another tile can pass into a bin in this tile, I like to call these hairy tiles). This lets us find edges that cross tiles boundaries (one of the problems noted above). Finally we employed one more trick to make the indexes easier to use. The graph is arrange so that you can traverse an edge in both the forward and reverse direction. To do this each edge is represented by two directed edges (one from A to B and one from B to A). Each of these directed edges has its own id. We only ever store one of these two ids in the bins the edges intersect. This cuts down on redundant shape decoding.

So what does the current implementation look like (v2?)? Well, after the indexes were added to the tiles we went to work using them. We wrote a little utility that lets you iterate (kind of like a python generator) over the bins in a closest first fashion (ie regarless of the bins containing tile). Basically we grab this closest bin first generator, seed it with the input point and iterate over the bins it passes us. If the bin is empty we move on, if its not, we decode the shapes of each edge and see if any edge was closer to this point than the current known closest. We terminate when either we've searched too far without finding anything or the bin we are about to look in is farther away than our current known closest edge. Since we look at bins in closest first order we shouldnt be able to find anything closer if the closest point in the bin is further than the current known closest edge.

This implementation is small, it prunes a lot of search space, it doesn't find the wrong edge (in terms of closeness), it can find things in tiles not containing the input coordinates and of course its way fast (media 4ms across NYC). It does cost in data size but it was measured as a less than %5 increase across the planet tileset.


#### What's next? ####

islands of connectivity
edge scoring vs filtering
fuzzy matching with expanded radius
z level concerns for dragging


### Benchmark ###

TODO:
