# PlanarPhysics

This is an engine for doing physics in the plane using geometric algebra.

![alt text](https://raw.githubusercontent.com/spencerparkin/PlanarPhysics/main/screenshot.jpg)

The primary focus of this engine is the simulation of the motion of bodies through space, and their collisions.  There are three basic types so far...

1. Walls,
2. Balls,
3. Convex polygons.

Walls are static.  They don't move, but you can bump into them.  Balls have position, but no orientation.  (I'm going to change this and given them angular velocity as well.)  And convex polygons are rigid bodies with position, orientation, linear velocity and angular velocity.