package com.hazardcook.dynamic_convex_pathfinding;
import java.util.List;

/**
 * Used to wrap a few functions of a world of a rigid body dynamics engine such as Box2D or dyn4j needed for any algorithms in {@link PathFinder}
 * @author nathan titus
 * @version 1.0
 *
 */
public interface WorldWrapper {
	
	/**
	 * Performs a raycast in a 2D rigid body world from a start point to an end point, returning the bodies hit by the raycast
	 * @param start starting point of the raycast
	 * @param end ending point of the raycast
	 * @return a {@link List} of {@link BodyWrapper}s that are sorted in increasing order of closeness to the start of the raycast
	 */
	public List<BodyWrapper> raycast(Vec2 start, Vec2 end);
	
	/**
	 * Performs a raycast in a 2D rigid body world from a start point to an end point against a particular body in that world
	 * @param start starting point of the raycast
	 * @param end ending point of the raycast
	 * @param body a {@link BodyWrapper} in the world
	 * @return whether or not the raycast hit the body, true if yes and false if no
	 */
	public boolean raycast(Vec2 start, Vec2 end, BodyWrapper body);
	
	/**
	 * Detects whether any bodies contain this point
	 * @param point the point to check
	 * @return whether any bodies contain this point, true if yes and false if no
	 */
	public boolean detect(Vec2 point);
	
}
