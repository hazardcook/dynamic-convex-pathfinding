package com.hazardcook.dynamic_convex_pathfinding;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * An implementation of a {@link PathFinder} that uses an algorithm developed by Nathan Titus to find a (generally short and accurate) {@link Path} from a start to an end {@link Vec2} given an implementation of a {@link BodyWrapper} 
 * @author nathan titus
 * @version 1.0
 *
 */
public class TitusPathFinder implements PathFinder{

	@Override
	public Path shortestPath(Vec2 start, Vec2 end, WorldWrapper world) {
		PathTree tree = new PathTree(start, end, world);
		return tree.shortestPath();
	}

	/**
	 * Not a traditional tree but helps to think of the way the paths branch as one.
	 * Able to find a {@link Path} given a start and end {@link Vec2} and an implementation of a {@link WorldWrapper}
	 * @author nathan titus
	 * @version 1.0
	 *
	 */
	private class PathTree {
		public static final int 
			DEFAULT_MAX_VISITS = 2
		;
		public static final double 
			DEFAULT_ANGULAR_GRAIN = Math.PI/18,
			DEFAULT_MOVEMENT_GRAIN = 1.0/15.0
		;
		
		private ArrayList<PathBranch> branchQueue = new ArrayList<PathBranch>();
		
		int maxVisits;
		double angularGrain;
		double movementGrain;

		WorldWrapper world;
		Vec2 start, end;
		
		/**
		 * Constructor that uses default values for maxVisits, angularGrain, and movementGrain
		 * @param start the starting location of the path
		 * @param end the ending location of the path
		 * @param world the world this path is searching in
		 */
		public PathTree(Vec2 start, Vec2 end, WorldWrapper world){
			maxVisits = DEFAULT_MAX_VISITS;
			angularGrain = DEFAULT_ANGULAR_GRAIN;
			movementGrain = DEFAULT_MOVEMENT_GRAIN;
			this.start = start;
			this.end = end;
			this.world = world;
		}
		
		/**
		 * Constructor used to specify custom parameters for this tree
		 * @param start the starting location of the path
		 * @param end the ending location of the path
		 * @param world the world this path is searching in
		 * @param maxVisits maximum number of times a convex may be worked around by a path before the path gives up
		 * @param angularGrain the angle by which the rotation step of the algorithm iteratively rotates a segment of a path (0 < x < 90). Increase for better performance and less accuracy. For best results, 1 < x 15
		 * @param movementGrain the amount by which the movement step of the algorithm moves a prospective location around as a fraction of distance from that location to the previous one (0 < x <= 1). Increase to gain performance and lose accuracy. For best results, decrease as size of convexes increases relative to distance from start to end point

		 */
		public PathTree(Vec2 start, Vec2 end, WorldWrapper world, int maxVisits, double angularGrain, double movementGrain){
			this.maxVisits = maxVisits;
			this.angularGrain = angularGrain;
			this.movementGrain = movementGrain;
		}
		
		/**
		 * Queues a {@link PathBranch} for a run of {@link#branchShortestPath()}
		 * @param branch the branch to queue
		 */
		public void queueForBranching(PathBranch branch){
			branchQueue.add(branch);
		}
		
		/**
		 * Finds a shortest path from the tree's start point to the end point. Uses the WorldWrapper to determine where the path
		 * is intersecting any convex bodies and adjusts the path accordingly.
		 * @return returns a {@link Path} which contains a list of {@link Vec2} locations in increasing order from start to end
		 */
		public Path shortestPath(){
			ArrayList<Vec2> startingPath = new ArrayList<Vec2>();
			startingPath.add(start);
			startingPath.add(end);
			PathBranch current = new PathBranch(this, startingPath, 0, new HashMap<BodyWrapper, Integer>());
			while(!current.branchShortestPath()){
				current = branchQueue.get(0);
				branchQueue.remove(0);
			}
			return new Path(current.locations);
		}
		
		/**
		 * Holds data and main logic required to run the path finding algorithm in {@link TitusPathFinder}
		 * @author nathan titus
		 * @version 1.0
		 */
		private class PathBranch {
			public PathTree tree;
			
			public ArrayList<Vec2> locations = new ArrayList<Vec2>();
			public HashMap<BodyWrapper, Integer> visitedBodies = new HashMap<BodyWrapper, Integer>();
			
			private int currentLocation;
			
			/**
			 * Constructor for a {@link PathBranch}
			 * @param tree the tree this path is a part of
			 * @param locations the list of locations this path has
			 * @param currentLocation the current location to be examined in the algorithm
			 * @param visitedBodies map of convexes to the number of times they have already been visited in the algorithm
			 */
			public PathBranch(PathTree tree, ArrayList<Vec2> locations, int currentLocation, HashMap<BodyWrapper, Integer> visitedBodies){
				this.tree = tree;
				this.locations = locations;
				this.currentLocation = currentLocation;
				this.visitedBodies.putAll(visitedBodies);
			}
			
			/**
			 * Increments the number of times the body has been visited and tells whether that body can be visited at this time
			 * @param body the {@link BodyWrapper} being visited (by the path searching algorithm)
			 * @return whether or not this body can be visited again, true if yes and false if no
			 */
			public boolean visit(BodyWrapper body){
				Integer visits = visitedBodies.get(body);
				if(visits == null){
					visitedBodies.put(body, (visits = 0));
				}
				visitedBodies.put(body, visits + 1);
				if(visits > maxVisits){
					return false;
				}
				return true;
			}
			
			/**
			 * Runs an iteration of the algorithm and queues any branches it creates in the tree
			 * @return whether or not this branch represents a found path yet, true if yes and false if no
			 */
			public boolean branchShortestPath(){
				/*
				 * Get a list of bodies in the world that cross this section of the path
				 */
				List<BodyWrapper> bodies = tree.world.raycast(locations.get(currentLocation), locations.get(currentLocation + 1));
				/*
				 * Only concerned with the first body crossed in this branch
				 */
				BodyWrapper firstBody = bodies.get(0);
				/*
				 * If there was no body, there is no more branching to be done between the second to last and
				 * last locations in the path. This path is good
				 */
				if(firstBody == null){
					return true;
				}
				/*
				 * If there was a body but it has been visited the max number of
				 * times, end this branch. This prevents bad paths from continuing
				 */
				else if(!visit(firstBody)){
					return false;
				}
				/*
				 * If there was a body and it is free to continue, recurse with
				 * two new branches reaching around the body
				 */
				else {
					/*
					 * Setup step
					 * 
					 * Create two new end points from next location. One will be rotated clockwise 
					 * around the convex, and the other counter-clockwise around the convex
					 * 
					 * Get the current location and next locations from the list
					 */
					
					Vec2 current = locations.get(currentLocation);
					Vec2 next = locations.get(currentLocation + 1);
					Vec2 clockwise = next.copy();
					Vec2 counter = next.copy();
					
					/*
					 * Rotate step
					 * 
					 * Rotate the two end points clockwise and counter-clockwise around
					 * the current point until raycasts between the current point and two end
					 * points no longer cross the convex. angularGrain is used here
					 * for the amount of rotation between iterations
					 */
					boolean done = false;
					while(!done){
						/*
						 * Rotate the clockwise point around the current point by angularGrain.
						 * If the raycast between the current point and clockwise point no longer
						 * hits the convex from earlier, rotating is finished
						 */
						clockwise.rotate(-tree.angularGrain, current);
						if(tree.world.raycast(current, clockwise, firstBody)){
							done = true;
						}
					}
					done = false;
					while(!done){
						/*
						 * Rotate the counter-clockwise point around the current point by angularGrain.
						 * If the raycast between the current point and counter-clockwise point no longer
						 * hits the convex from earlier, rotating is finished
						 */
						counter.rotate(tree.angularGrain, current);
						if(tree.world.raycast(current, counter, firstBody)){
							done = true;
						}
					}
					
					/*
					 * End point regression or progression step
					 * 
					 * Move the clockwise and counter-clockwise points toward the current point by movementGrain
					 * until the raycast from the new clockwise and counter-clockwise points to the end point
					 * hits the convex or the points end up inside any convex, then take it a step back. 
					 * This keeps a tight path around the convex
					 */
					
					/*
					 * Initialize a movement vector that when added to the clockwise point moves it by 
					 * the distance from clockwise to the start point multiplied by movementGrain toward
					 * the current point
					 */
					Vec2 moveClockwise = current.copy().sub(clockwise).mul(tree.movementGrain);
					done = false;
					while(!done){
						/*
						 * Incrementally move the clockwise point toward the current point. Then, if a raycast
						 * from the clockwise point to the end point crosses the convex, or the point has
						 * been moved inside any other convex, move back an iteration and the movement is finished
						 */
						clockwise.add(moveClockwise);
						if(tree.world.raycast(clockwise, next, firstBody) || tree.world.detect(clockwise)){
							done = true;
							clockwise.sub(moveClockwise);
						}
					}
					/*
					 * Initialize a movement vector using the same logic above applied to the counter-clockwise 
					 * point
					 */
					Vec2 moveCounter = current.copy().sub(counter).mul(tree.movementGrain);
					done = false;
					while(!done){
						/*
						 * Same steps as above but for the counter-clockwise point
						 */
						counter.add(moveCounter);
						if(tree.world.raycast(counter, next, firstBody) || tree.world.detect(counter)){
							done = true;
							counter.sub(moveCounter);
						}
					}
					
					/*
					 * Branch step
					 * 
					 * At this step both the clockwise point and counter-clockwise points should not be inside
					 * any convexes, and the line from both clockwise or counter-clockwise to the end point and
					 * from the current point to the clockwise and counter-clockwise points should not be obstructed 
					 * by the original convex. Now we must create new branches from those two points to the end point
					 * 
					 * Another outcome is that the line from start to either of the new points may be blocked
					 * by some other convex now. If this is the case, the new branches will need to repeat these steps
					 * at the current location
					 * 
					 * Create two new paths with the added respective locations of the clockwise point and
					 * counter-clockwise point. If there is no object obstructing this part of the path,
					 * increment the current location for the next iteration of the paths
					 */
					ArrayList<Vec2> clockwiseLocations = new ArrayList<Vec2>();
					ArrayList<Vec2> counterLocations = new ArrayList<Vec2>();
					
					/*
					 * Copy the locations of this path into the two new paths and insert the new clockwise and 
					 * counter-clockwise point locations into their positions
					 */
					clockwiseLocations.addAll(locations);
					clockwiseLocations.add(currentLocation + 1, clockwise);
					counterLocations.addAll(locations);
					counterLocations.add(currentLocation + 1, counter);
					
					/*
					 * If there is still a convex intersecting the line formed by this location and the next,
					 * keep the current location the same so the branch can attempt to get around it in the
					 * next iteration. Otherwise move onto the next location
					 */
					int clockwiseCurrent = currentLocation, counterCurrent = currentLocation;
					if(tree.world.raycast(current, clockwise).isEmpty()){
						clockwiseCurrent += 1;
					}
					if(tree.world.raycast(current, counter).isEmpty()){
						counterCurrent += 1;
					}
					
					/*
					 * Create the new path branches and queue them in the tree
					 */
					PathBranch clockwisePath = new PathBranch(tree, clockwiseLocations, clockwiseCurrent, visitedBodies);
					PathBranch counterPath = new PathBranch(tree, counterLocations, counterCurrent, visitedBodies);
					tree.queueForBranching(clockwisePath);
					tree.queueForBranching(counterPath);
					
					/*
					 * Didn't finish finding the path on this branch so return false
					 */
					return false;
				}
			}
		}
	}
}
