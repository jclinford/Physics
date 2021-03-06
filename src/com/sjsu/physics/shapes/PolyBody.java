package com.sjsu.physics.shapes;

import java.awt.Polygon;
import java.awt.Rectangle;
import java.util.ArrayList;

import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;

/**
 * A polygon rigidBody. Described by a list of normals (vector2), a polygon and a location
 */
public class PolyBody extends RigidBody
{
	private Polygon polygon = new Polygon();
	private ArrayList<Vector2> normals;

	public PolyBody(Polygon p, Vector2 cen)
	{
		setType(BodyType.POLYGON);
		setCenter(cen);
		setPolygon(p);
	}

	public PolyBody(Polygon p, float x, float y)
	{
		setType(BodyType.POLYGON);
		setCenter(x, y);
		setPolygon(p);
	}

	/** Polygon's vertices must be listed in counterclockwise order */
	public void setPolygon(Polygon p)
	{
		float edgeCount = 0;

		// check that vertices are counterclockwise
		for (int i = 0; i < p.npoints; i++)
		{
			Vector2 v1 = new Vector2(p.xpoints[i], p.ypoints[i]);
			Vector2 v2 = new Vector2(p.xpoints[(i + 1) % p.npoints], p.ypoints[(i + 1) % p.npoints]);

			// sum over edges (x2 - x1) * (y2 + y1)
			edgeCount += (v2.x() - v1.x()) * (v2.y() + v1.y());
		}

		// if edgeCount is negative then we are counterclockwise
		if (edgeCount > 0)
			throw new RuntimeException("Polygon is not counterclockwise");

		polygon.npoints = p.npoints;
		polygon.xpoints = p.xpoints;
		polygon.ypoints = p.ypoints;
		polygon.invalidate();

		calcBounds();
		calcNormals();
		calculateMoment();
	}

	/** Calculate the bounds based on our body's polygon */
	private void calcBounds()
	{
		bounds.setHalfHeight(polygon.getBounds().height / 2);
		bounds.setHalfWidth(polygon.getBounds().width / 2);
	}

	/** Calculate all the normals for this body's Polygon */
	private void calcNormals()
	{
		if (polygon == null)
			throw new RuntimeException("Polygon not associated with this PolyBody");


		normals = new ArrayList<Vector2>(polygon.npoints + 1);
		for (int i = 0; i < polygon.npoints; i++)
		{
			int j = (i + 1) % polygon.npoints;

			Vector2 iV = new Vector2(polygon.xpoints[i], polygon.ypoints[i]);
			Vector2 jV = new Vector2(polygon.xpoints[j], polygon.ypoints[j]);

			// edge JV - IV, so normal is a 270 deg rotation, we are assuming
			// polygons are ordered counter-clockwise
			normals.add(jV.subtractBy(iV).rotate270());
		}

		if (normals.size() != polygon.npoints)
			throw new ArrayIndexOutOfBoundsException("Normals.size != polygon.npoints");
	}

	/** Checks if a point is within this polygon */
	public boolean contains(Vector2 point)
	{
		point = this.state().TransformIntoSpaceOf(point);
		ArrayList<Vector2> polyVertices = this.verticesWorld();

		for (int i = 0; i < polyVertices.size(); i++)
		{
			Vector2 dist = point.subtractBy(polyVertices.get(i));
			if (dist.dot(polyVertices.get(i)) > 0)
				return false;	// separating axis found
		}
		return true;
	}

	/** Add a force at the point which is given in bodyCoords */
	public void addForceAtBodyPoint(Vector2 force, Vector2 point)
	{
		addForceAtPoint(force, point.addTo(center()));
	}

	/** Add a force at the point which is given in worldCoords */
	public void addForceAtPoint(Vector2 force, Vector2 point)
	{
		Vector2 pt = point;
		pt = pt.subtractBy(center());

		this.addForce(force);
		this.addTorque(pt.dot(force));
	}

	/** Return vertices in bodyCoords */
	public ArrayList<Vector2> vertices()
	{
		ArrayList<Vector2> polyPoints = new ArrayList<Vector2>(polygon.npoints + 1);

		for (int i = 0; i < polygon.npoints; i++)
		{
			Vector2 v = new Vector2(polygon.xpoints[i], polygon.ypoints[i]);
			polyPoints.add(v);
		}

		return polyPoints;
	}

	/** Return vertices coords in worldCoords (add center and rotate by orientation) */
	public ArrayList<Vector2> verticesWorld()
	{
		ArrayList<Vector2> polyPoints = new ArrayList<Vector2>(polygon.npoints + 1);

		for (int i = 0; i < polygon.npoints; i++)
		{
			Vector2 v = new Vector2(polygon.xpoints[i], polygon.ypoints[i]);
			v = v.rotate(this.orientation());
			v = v.addTo(this.center());
			polyPoints.add(v);
		}

		return polyPoints;
	}

	/** Number of vertices of our polygon */
	public int numVertices()
	{
		if (polygon != null)
			return polygon.npoints;
		return 0;
	}

	/** Return the polygon in bodyCoords */
	public Polygon polygon()
	{
		return polygon;
	}

	/** Return the polygon in world coords (add center and rotate by orientation) */
	public Polygon polygonWorld()
	{
		int[] xpointsWorld = new int[polygon.npoints];
		int[] ypointsWorld = new int[polygon.npoints];

		for (int i = 0; i < polygon.npoints; i++)
		{
			float cs = (float) Math.cos(this.orientation());
			float sn = (float) Math.sin(this.orientation());

			float x = polygon.xpoints[i] * cs - polygon.ypoints[i] * sn;
			float y = polygon.xpoints[i] * sn + polygon.ypoints[i] * cs;
			xpointsWorld[i] = (int) (x + center().x());
			ypointsWorld[i] = (int) (y + center().y());
		}

		Polygon p = new Polygon(xpointsWorld, ypointsWorld, polygon.npoints);
		return p;
	}

	/** Rotate the vertices about the center by angle rad */
	public Polygon rotatePolygon(float rad)
	{
		float s = (float) Math.sin(rad);
		float c = (float) Math.cos(rad);
		int xPoints[] = new int[polygon.npoints];
		int yPoints[] = new int[polygon.npoints];
		int nPoints = 0;

		for (nPoints = 0; nPoints < polygon.npoints; nPoints++)
		{
			int x = polygon.xpoints[nPoints];
			int y = polygon.ypoints[nPoints];

			xPoints[nPoints] = (int) (x * c - y * s);
			yPoints[nPoints] = (int) (x * s + y * c);
		}

		Polygon p = new Polygon(xPoints, yPoints, nPoints);
		return p;
	}

	/** Returns the normal for the edge between vertex i and vertex i+1 */
	public Vector2 normal(int i)
	{
		return normals.get(i);
	}

	/** Returns the normal in worldCoords */
	public Vector2 normalWorld(int i)
	{
		Vector2 normal = normals.get(i);
		return normal.rotate(orientation());
	}

	@Override
	public void rotateBy(float rad)
	{
		// rotate the state matrix
		super.rotateBy(rad);

		// update the boundingBox
		Rectangle p = this.rotatePolygon(orientation()).getBounds();
		bounds.setHalfHeight(p.height / 2);
		bounds.setHalfWidth(p.width / 2);
	}

	@Override
	public void calculateMoment()
	{
		if (polygon == null)
			return;

		if (inverseMass() == 0)
		{
			inverseMomentOfInertia = Globals.ZERO_VECTOR;
			return;
		}

		/* Moment of inertia for a box = ( bh^3 ) / 12 */
		float width = bounds.halfWidth() * 2;
		float height = bounds.halfHeight() * 2;
		inverseMomentOfInertia.set(12 / (width * height * height * height), 12 / (height * width
				* width * width));
	}

	@Override
	public String toString()
	{
		String s = "Center: " + center() + "  Velocity:" + velocity();

		for (int i = 0; i < polygon.npoints; i++)
		{
			float x = polygon.xpoints[i] + center().x();
			float y = polygon.ypoints[i] + center().y();
			s += "  Vertex " + i + ": (" + x + ", " + y + ")";
		}

		return s;
	}
}
