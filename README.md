# dynamic-convex-pathfinding
An API that finds shortest paths in a dynamic world of 2D convex shapes. Ideal for use with 2D physics engines like Box2D or dyn4j.

Why?
I'm using the dyn4j physics engine in some 2D games projects that I am working on and wanted
to be able to do pathfinding on a world of random 2D convexes moving around constantly. I
didn't want to generate navigation meshes because of the dynamic world. I decided to make this
new algorithm after doing some research and finding nothing quite like it.

How?
The algorithm works in a few steps.
First, there needs to be a start and end point, of course. These can be any floating point value.
Next, a "root" path is made, which is simply a list containing only the start and end points.
Now here is the idea that led me to make this algorithm. It seemed to me that if I were to just bend
this line around any convexes that cross its path I would eventually have a way from my start point
to my end point. So that's what this algorithm does.

It loops until it is finished making the path. In each iteration, it finds the first location of
the path that crosses a convex with the line formed by it and the next location. It then copies
the next location twice and rotates the new points clockwise and counter-clockwise around the convex
until the lines made from the current location and those two points no longer intersects the convex.

It then moves those new points toward the current location until the line formed by the new points and
the next location intersect the convex, then they are moved back a step. This keeps the path "tight" 
and ensures that none of the lines from the current location to the two new points, and from the two new 
points to the next location intersect the convex.

Now the two points are moved backwards while they are contained inside any convex in the world, until
they are in an open space. This is necessary if the tightening of the path moved those points into a
convex, or if they found themselves in a convex after the rotation to begin with and tightening the path
never freed them.

Next, we see if there are any new convexes intersecting the line from our current location to these two
new points, even after all this. We got the path out of the way of the one convex but may have put it
into a new convex. We actually might do this multiple times with the same convexes, so there is a max
number of times a convex may be visited this way on a per path basis. If this happens the path does not 
branch into new paths to cull the number of paths working. If either of those paths does intersect a convex
still, we keep the current location set for those paths as the current location so they can resolve
that intersection before moving on.

Finally, the paths are constructed that include the clockwise and counter-clockwise rotated points
and they are put into a queue. This queue contains all new paths constructed this way and finishes all
paths on one level of this tree before moving to the next.

If a path finds that there is no convex between the current location it is checking and the next location,
this actually means the algorithm is done! Since we ensure that all parts of the path connected before the
current location are not blocked by any convexes, the next location will only be free of a blocking convex
if the last iteration finally got the path around a convex that didn't lead to another collision.

This algorithm is by no means perfect. It can also explode if too high of a max iterations for visiting
bodies is chosen. The path generally finds close to the shortest path possible, and does so quickly if
tuned right. The default values I included worked well for me. It should also be noted that this algorithm
works better when there are fewer convexes. The more space there is to work with the more likely it will be
that the algorithm finishes quickly. If there are only tiny gaps between convexes and the actual shortest
is actually this enormous winding beast then the algorithm will most likely fail after visiting the same
bodies too many times. Trying to increase the number of times the algorithm can visit the same body will
do the opposite: branch into so many paths no progress can be made by the computer.

That said, I found this algorithm was able to find a path every frame during a 60 FPS simulation in a world
filled with hundreds of random rectangles and circles moving around and bumping into each other, and the path
was of good quality. Enough for an enemy AI in a game by far.
