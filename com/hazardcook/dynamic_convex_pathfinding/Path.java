package com.hazardcook.dynamic_convex_pathfinding;

import java.util.List;

/**
 * A path from a start to an end {@link Vec2}. Starting location is at index 0 of locations and locations are in order until the last {@link Vec2}
 * @author nathan titus
 * @version 1.0
 *
 */
public class Path {
	public List<Vec2> locations;
	
	/**
	 * Create a {@link Path} from a list of locations
	 * @param locations the list of locations to hold in this path
	 */
	public Path(List<Vec2> locations){
		this.locations = locations;
	}
}
