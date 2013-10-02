package com.sjsu.physics.utils;

import java.awt.Rectangle;

/* 
 * Globals and constants used throughout the entire engine 
 * Unless otherwise stated we are assuming MGS units
 */
public class Globals 
{
	/* Constants for math / physics */
	public static final int INFINITY = 2147483646;			// 'inifinity' or the max int value
	public static final float EPSILON = .000001f;
	public static final float DEFAULT_RESTITUTION = 1f;		// Restitution used for collisions by default
	public static final float g = (float) 9.807 * 10;
	public static final float DEFAULT_GRAVITY = (float) 1 * 10;
	public static final float DISTANCE_TO_REMOVE = 3;		// at > DISTANCE_TO_REMOVE penetration we will remove the contact
	public static final float SLEEP_EPSILON = .5f;			// minimum velocity needed to  put a body to sleep
	public static final Vector2 ZERO_VECTOR = new Vector2(0, 0);

	
	/* globals for collision detection */
	public static final float ANGULAR_TOLLERANCE = 5f;
	public static final int MAX_ANGULAR_CAST_ITERATIONS = 20;


	/* Number of processors/threads to use */
	public static final int NUM_PROCESSORS = 4;
	
	/* Quadrant to number values.. used in quadtree */
	public static final int TOP_LEFT = 0;
	public static final int TOP_RIGHT = 1;
	public static final int BOTTOM_LEFT = 2;
	public static final int BOTTOM_RIGHT = 3;
	public static final int MAX_QUADTREE_DEPTH = 1;
	public static final int MAX_QUADTREE_CHILDREN = 15;
	
	/* Defualt number of bodies for arrayList in World */
	public static final int DEFAULT_BODY_SIZE = 500;
	
	
	/* Defaults for rigid body geometries */
	public static final float DEFAULT_CIRCLE_RADIUS = 4;
	public static final float DEFAULT_MASS = 1;
	
	
	/* Default number of iterations to take before bailing for contact resolving */
	public static final int CONTACT_SOLVER_DEFAULT_ITERATIONS = 10;
	
	
	/* Conversion factors */
	public static final float NANOSEC_TO_SECONDS = .000000001f;		// 1nanosecond = 1.0e-9sec
	public static final float NANOSEC_TO_MILLISEC = .000001f;		// 1 nanosecond = 1.0e-6mS
	public static final float MAX_TIME_STEP = 1/1f;				// Maximum step time, so things dont get out of hand
	public static final float MIN_TIME_STEP = 1/100000f;				// Minimum step time, so things always progress
	
	
	
	
	/*============ For example Game ===========*/
	/* The game rect or entire world */
	public static final int MAX_GAME_WIDTH = 1000;
	public static final int MAX_GAME_HEIGHT = 1000;
	public static final Rectangle GAME_RECT = new Rectangle(0, 0, MAX_GAME_WIDTH, MAX_GAME_HEIGHT);
	public static final boolean DRAW_QUADTREE = true;
}