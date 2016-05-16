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
	
	PathTree tree;
	public void initIterPath(Vec2 start, Vec2 end, WorldWrapper world){
		tree = new PathTree(start, end, world);
	}
	public Path iterShortestPath(){
		return tree.iterShortestPath();
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
			DEFAULT_ANGULAR_GRAIN = Math.PI/36.7,
			DEFAULT_MOVEMENT_GRAIN = 1.0/45.7
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
			this.start = start;
			this.end = end;
			this.world = world;
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
		 * @return returns a {@link Path} which contains a list of {@link Vec2} locations in increasing order from start to end or null if a path wasn't found
		 */
		public Path shortestPath(){
			ArrayList<Vec2> startingPath = new ArrayList<Vec2>();
			startingPath.add(start);
			startingPath.add(end);
			PathBranch current = new PathBranch(this, startingPath, 0, new HashMap<BodyWrapper, Integer>());
			while(!current.branchShortestPath()){
				if(branchQueue.isEmpty()){
					return null;
				} else {
					current = branchQueue.get(0);
					branchQueue.remove(0);
				}
			}
			return new Path(current.locations);
		}
		
		
		boolean finished = true;
		/**
		 * Runs one iteration of the algorithm on the next path branch in the queue
		 * @return the {@link Path} so far of the {@link PathBranch}
		 */
		public Path iterShortestPath(){
			if(branchQueue.isEmpty() && finished){
				ArrayList<Vec2> startingPath = new ArrayList<Vec2>();
				startingPath.add(start);
				startingPath.add(end);
				PathBranch current = new PathBranch(this, startingPath, 0, new HashMap<BodyWrapper, Integer>());
				branchQueue.add(current);
				finished = false;
			}
			else if (branchQueue.isEmpty()){
				finished = true;
				return null;
			}
			PathBranch current = branchQueue.get(0);
			branchQueue.remove(0);
			current.branchShortestPath();
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
				visitedBodies.put(body, (visits += 1));
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
				BodyWrapper firstBody = bodies.isEmpty() ? null : bodies.get(0);
				/*
				 * If there was no body and the next location is the end, pathfinding has finished. If it's not the end,
				 * iterate the current location and requeue this path.
				 */
				if(firstBody == null){
					/*
					 * If the next location is the end point the path is finished
					 */
					if(currentLocation + 1 == locations.size() - 1){
						return true;
					/*
					 * Otherwise increment the current location and requeue this path
					 */
					} else {
						currentLocation += 1;
						tree.queueForBranching(this);
						return false;
					}
				}
				/*
				 * If there was a body but it has been visited the max number of
				 * times, end this branch. This prevents bad paths from continuing
				 */
				else if(!visit(firstBody)){
					return false;
				}
				/*
				 * If either the start or end point is inside a body, no path can be found by continuing this path
				 * at this point
				 */
				else if(world.detect(locations.get(currentLocation)) || world.detect(locations.get(currentLocation + 1))){
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
					
					rotateStep(current, clockwise, counter, firstBody);
					slidingStep(current, clockwise, counter, next, firstBody);
					branchStep(current, clockwise, counter);
					
					/*
					 * Didn't finish finding the path on this branch so return false
					 */
					return false;
				}
			}
			
			
			/*
			 * Rotate step
			 * 
			 * Rotate the two end points clockwise and counter-clockwise around
			 * the current point until raycasts between the current point and two end
			 * points no longer cross the convex. angularGrain is used here
			 * for the amount of rotation between iterations
			 */
			private void rotateStep(Vec2 current, Vec2 clockwise, Vec2 counter, BodyWrapper firstBody){
				
				while(tree.world.raycast(current, clockwise, firstBody)){
					/*
					 * Rotate the clockwise point around the current point by angularGrain.
					 * If the raycast between the current point and clockwise point no longer
					 * hits the convex from earlier, rotating is finished
					 */
					clockwise.rotate(-tree.angularGrain, current);
				}
				while(tree.world.raycast(current, counter, firstBody)){
					/*
					 * Rotate the counter-clockwise point around the current point by angularGrain.
					 * If the raycast between the current point and counter-clockwise point no longer
					 * hits the convex from earlier, rotating is finished
					 */
					counter.rotate(tree.angularGrain, current);
				}
			}
			
			/*
			 * End point sliding step
			 * 
			 * Move the clockwise and counter-clockwise points toward the current point by movementGrain
			 * until the raycast from the new clockwise and counter-clockwise points to the end point
			 * hits the convex, then take it a step back. This keeps a tight path around the convex
			 */
			private void slidingStep(Vec2 current, Vec2 clockwise, Vec2 counter, Vec2 next, BodyWrapper firstBody){
				Vec2 moveClockwise = current.copy().sub(clockwise).mul(tree.movementGrain);
				Vec2 moveCounter = current.copy().sub(counter).mul(tree.movementGrain);
				slideTowardFirstConvex(current, clockwise, moveClockwise, counter, moveCounter, next, firstBody);
				slideOutOfOtherConvexes(clockwise, moveClockwise, counter, moveCounter);
			}
			
			/*
			 * Slides the clockwise and counter-clockwise (middle) points toward the current location
			 * until the segment from the middle points to the next point intersect the convex
			 * being worked around, then moves them back one step.
			 */
			private void slideTowardFirstConvex(Vec2 current, Vec2 clockwise, Vec2 moveClockwise, 
					Vec2 counter, Vec2 moveCounter, Vec2 next, BodyWrapper firstBody){
				/*
				 * While the convex is not intersecting the segment from the middle point to the next
				 * point, move the middle point toward the current point. Then move it back one step
				 * so it is not intersecting the convex but is still close
				 */
				while(!tree.world.raycast(clockwise, next, firstBody)){
					clockwise.add(moveClockwise);
				}
				clockwise.sub(moveClockwise);
				
				/*
				 * Same process for the counter point as the clockwise point
				 */
				while(!tree.world.raycast(counter, next, firstBody)){
					counter.add(moveCounter);
				}	
				counter.sub(moveCounter);
			}
			
			/*
			 * Slides the middle points away from the current location until the middle points are
			 * not inside any convexes. If sliding toward the current convex put them inside any
			 * other convexes, or the rotation step before that put the point inside any convex
			 * this will get the points out.
			 */
			private void slideOutOfOtherConvexes(Vec2 clockwise, Vec2 moveClockwise, Vec2 counter, Vec2 moveCounter){
				/*
				 * Move clockwise until it is out of any convexes
				 */
				while(tree.world.detect(clockwise)){
					clockwise.sub(moveClockwise);
				}
				/*
				 * Move counter until it is out of any convexes
				 */
				while(tree.world.detect(counter)){
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
			private void branchStep(Vec2 current, Vec2 clockwise, Vec2 counter){
				/*
				 * Copy the locations of this path into two new paths and insert the new clockwise and 
				 * counter-clockwise point locations into their positions
				 */
				ArrayList<Vec2> clockwiseLocations = new ArrayList<Vec2>();
				ArrayList<Vec2> counterLocations = new ArrayList<Vec2>();
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
			}
		}
	}
}
