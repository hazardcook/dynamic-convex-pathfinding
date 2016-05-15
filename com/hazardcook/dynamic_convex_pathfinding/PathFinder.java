package com.hazardcook.dynamic_convex_pathfinding;

/**
 * An interface for classes that can find a shortest path given a start and end {@link Vec2} and a {@link WorldWrapper} wrapping the world of a 2D physics engine
 * @author nathan titus
 * @version 1.0
 *
 */
public interface PathFinder {

	public Path shortestPath(Vec2 start, Vec2 end, WorldWrapper world);
	
}
