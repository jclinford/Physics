package com.sjsu.physics.shapes;

import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;

public class Circle extends RigidBody
{
	public Circle()
	{
		// All objects have been instantiated to zero/defaults in RigidBody
		
		setType(BodyType.CIRCLE);
		setRadius(Globals.DEFAULT_CIRCLE_RADIUS);
	}
	
	public Circle(float x, float y)
	{
		setType(BodyType.CIRCLE);
		setRadius(Globals.DEFAULT_CIRCLE_RADIUS);
		setCenter(x, y);
	}
	
	public Circle(float x, float y, float rad)
	{
		setType(BodyType.CIRCLE);
		setRadius(rad);
		setCenter(x, y);
	}
	
	public Circle(Vector2 cen, float rad)
	{
		setType(BodyType.CIRCLE);
		setRadius(rad);
		setCenter(cen);
	}
	
	public void setRadius(float rad)
	{
		bounds.setHalfHeight(rad);
		bounds.setHalfWidth(rad);
	}

	@Override
	public void calculateMoment()
	{
		float moment = (float) (4 / ( Math.PI * Math.pow(bounds.radius(), 4)));
		inverseMomentOfInertia.set(moment, moment);
	}
}
