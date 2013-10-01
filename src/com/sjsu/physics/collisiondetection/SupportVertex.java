package com.sjsu.physics.collisiondetection;

import com.sjsu.physics.utils.Vector2;


/* Support vertex used for collision detection */
public class SupportVertex 
{
	public Vector2 vector;
	public int index;
	
	public SupportVertex(Vector2 v, int i)
	{
		vector = v.getCopy();
		index = i;
	}
}
