package com.hazardcook.dynamic_convex_pathfinding;

/**
 * Simple implementation of a 2 dimensional vector with necessary functions for algorithms in {@link PathFinder}
 * @author nathan
 *
 */
public class Vec2 {

	public double x, y;
	
	/**
	 * Constructor that initializes the x and y components of this vector
	 * @param x the x component of the vector
	 * @param y the y component of the vector
	 */
	public Vec2(double x, double y){
		this.x = x;
		this.y = y;
	}
	
	/**
	 * Subtracts the x and y components of another vector from this vector
	 * @param other the {@link Vec2} being subtracted from this
	 * @return this {@link Vec2}
	 */
	public Vec2 sub(Vec2 other){
		x -= other.x;
		y -= other.y;
		return this;
	}
	
	/**
	 * Adds the x and y components of another vector to this vector
	 * @param other the {@link Vec2} being added to this
	 * @return this {@link Vec2}
	 */
	public Vec2 add(Vec2 other){
		x += other.x;
		y += other.y;
		return this;
	}
	
	/**
	 * Divides the x and y components of this vector by the scalar value given
	 * @param scalar the value to divide by
	 * @return this {@link Vec2}
	 */
	public Vec2 div(double scalar){
		x /= scalar;
		y /= scalar;
		return this;
	}
	
	/**
	 * Multiplies the x and y components of this vector by scalar value given
	 * @param scalar the value to multiply by
	 * @return this {@link Vec2}
	 */
	public Vec2 mul(double scalar){
		x *= scalar;
		y *= scalar;
		return this;
	}
	
	/**
	 * Rotates this (@link Vec2) around the origin.
	 * @param theta angle of rotation in RADIANS
	 * @return this (@link Vec2)
	 */
	public Vec2 rotate(double theta){
		double cos = Math.cos(theta), sin = Math.sin(theta);
		x = x*cos - y*sin;
		y = x*sin + y*cos;
		return this;
	}
	
	/**
	 * Rotates this (@link Vec2) around the given pivot point
	 * @param theta angle of rotation in RADIANS
	 * @param pivot the point to rotate around
	 * @return this (@link Vec2)
	 */
	public Vec2 rotate(double theta, Vec2 pivot){
		x -= pivot.x;
		y -= pivot.y;
		rotate(theta);
		x += pivot.x;
		y += pivot.y;
		return this;
	}
	
	/**
	 * Creates a new {@link Vec2} object that has the same x and y component values as this vector
	 * @return a new {@link Vec2} with copied x and y components
	 */
	public Vec2 copy(){
		return new Vec2(x, y);
	}
}
