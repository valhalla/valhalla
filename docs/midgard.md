# Midgard

Midgard serves as a set of geographic/metric data structures and algorithms for use in essentially all other parts of valhalla. In keeping with the Norse mythological theme, the name Midgard was chosen as it is represents the world as humans know it. Since the software and classes within midgard deal mostly with the maths of our favorite spheroid, this seemed like a fitting name!

Midgard contains a set of various geometric data structures and support classes. It also includes algorithms which deal with things like: closest point to line, vector and matrix operations, polyline encoding, logging, projection, tiling, and culling. Midgard also include a variety of constants for time and distance conversions as well as angular measures.

## Components ##

What follows are some notable components of midgard.

### 2-D Points and Latitude, Longitude ###

Midgard includes classes to support two-dimensional points: either Cartesian x,y (Point2) or latitude, longitude (PointLL). Basic operations such as distance between 2 points are provided, with the PointLL providing arc-distance or distance along a spherical earth.

### 2-D Vectors ###

Midgard includes a 2-D vector class. This is a templated class so it can support either Cartesian coordinates or latitude, longitude coordinates. Basic 2-D vector operations such as dot products, scaling, normalization, projection, and angle computations are provided.

### Line Segments and Polylines ###

Midgard provides a class supporting 2-D line segments. This is a templated class so it can support either Cartesian coordinates or latitude, longitude coordinates. Various intersection methods, clipping methods, and distance methods are provided.

### 2-D Bounding Boxes ###

Midgard includes a class to support 2-D axis-aligned bounding boxes. This is a templated class so it can support either Cartesian coordinates or latitude, longitude coordinates. Methods to determine if points are inside a bounding box are provided as well as methods to find the intersection of 2 bounding boxes. Clipping operations to clip polylines to the bounding box are also provided.

A class supporting oriented or general 2-D bounding boxes is also provided.

### Ellipse ###

Midgard contains a class supporting methods to construct an ellipse, test if a line segment intersects the ellipse, test if an axis-aligned bounding box intersects, and test whether a point is within the ellipse. This is a templated class to work with Point2 (Euclidean x,y) or PointLL (latitude,longitude).

### Tiles ###

Midgard provides a class that provides a uniform (square) tiling system for a specified bounding box and tile size. This is a template class that works with Point2 (Euclidean x,y) or PointLL (latitude,longitude). A unique tile Id is assigned for each tile based on the following rules:
Tile numbers start at 0 at the min y, x (lower left)
Tile numbers increase by column (x,longitude) then by row (y,latitude)
Tile numbers increase along each row by increasing x,longitude.
This class contains methods for converting x,y or lat,lng into tile Id and vice-versa.  Methods for relative tiles (using row and column offsets) are also provided. Also includes a method to get a list of tiles covering a bounding box.

### Gridded Data ###

Midgard provides a class to store data in a gridded/tiled data structure. Contains methods to mark each tile with data using a compare operator. There is also a method to generate contour lines from isotile data. The contouring method is a derivation from the C code version of CONREC by Paul Bourke:
http://paulbourke.net/papers/conrec/

## Distance Approximation ###

Midgard provides a special class to perform distance approximation in latitude, longitude space. This class approximates distance (meters) between two points. This method is more efficient than using spherical distance calculations within the PointLL class. It computes an approximate distance using the pythagorean theorem with the meters of latitude change (exact) and the meters of longitude change at the "test point". Longitude is inexact since meters per degree of longitude changes with latitude. This approximation has very little error (less than 1%) if the positions are close to one another (within several hundred meters). Error increases at high (near polar) latitudes. This method will not work if the points cross 180 degrees longitude.

### Polyline Encoding ###

TODO:

### Logging ###

TODO:

### Util ###

midgard includes a variety of utility methods supporting : TODO

### Sequence ###

TODO:
