package com.sjsu.physics.utils;

public class Ray 
{
	static public float rayInterserctPlane(Vector2 a, Vector2 b, Vector2 p, Vector2 n)
	{
		float numerator = p.subtractBy(a).dot(n);
		float denom = b.subtractBy(a).dot(n);
		
		if (denom != 0)
			return numerator / denom;

		return Globals.INFINITY;
	}
	
	static private float rayIntersectInternal(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
	{
		// turn c->d into a plane
		Vector2 n = d.subtractBy(c).rotate270();
		
		float numerator = c.subtractBy(a).dot(n);
		float denom = b.subtractBy(a).dot(n);
		
		if (denom != 0)
			return numerator / denom;
		
		return Globals.INFINITY;
	}
	
	// Intersect two line segments (a->b and c->d)
	// Returns the time of intersection along a->b, or -1 if there is no intersection
	static public float rayIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
	{
		float abResult = rayIntersectInternal(a, b, c, d);
		if (abResult != Globals.INFINITY)
		{
			float cdResult = rayIntersectInternal(c, d, a, b);
			if (cdResult != Globals.INFINITY)
			{
				if (abResult >= 0 && abResult <= 1 && cdResult >= 0 && cdResult <= 1)
					return abResult;
			}
		}
		return -1;
	}

}
