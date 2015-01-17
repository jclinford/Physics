package com.sjsu.physics.collisiondetection;

import java.util.ArrayList;

import com.sjsu.physics.core.Contact;
import com.sjsu.physics.shapes.BoundingBox;
import com.sjsu.physics.shapes.Circle;
import com.sjsu.physics.shapes.PolyBody;
import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.shapes.RigidBody.BodyType;
import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;

/**
 * Performs fine collision detection.
 * No false positives nor false negatives.
 *
 * Uses exact checks to determine overlap, slow but accurate
 */
public class FineCollision
{
	/** Check for a collision between a and b, fill contact list with any contacts found */
	public static void getContactPoints(RigidBody a, RigidBody b, ArrayList<Contact> contacts)
	{
		// if both objects are infinity then we do nothing
		if (a.inverseMass() == 0 && b.inverseMass() == 0)
			return;

		// Early-out check to make sure boundingCircles are intersecting
		if (!boxesAreColliding(a, b))
			return;

		// make sure we don't already have this contact stored
		if (contactsContainContact(contacts, a, b))
			return;

		Contact contact = handleGenericCollision(a, b);
		if (contact != null)
			contacts.add(contact);

		return;
	}

	/** We handle different collisions differently based on input object types */
	private static Contact handleGenericCollision(RigidBody a, RigidBody b)
	{
		Contact contact = null;

		if (a.type() == BodyType.CIRCLE && b.type() == BodyType.CIRCLE)
			contact = circleCircle((Circle) a, (Circle) b);
		else if (a.type() == BodyType.CIRCLE && b.type() == BodyType.POLYGON)
			contact = circlePolygon((Circle) a, (PolyBody) b);
		else if (a.type() == BodyType.POLYGON && b.type() == BodyType.CIRCLE)
			contact = circlePolygon((Circle) b, (PolyBody) a);
		else if (a.type() == BodyType.POLYGON && b.type() == BodyType.POLYGON)
			contact = polygonPolygon((PolyBody) a, (PolyBody) b);
		else
			throw new IndexOutOfBoundsException("Unknown collision enum");

		return contact;
	}

	/** Check if our contact list contains body a and b already to avoid multithreading multiple adds */
	private static boolean contactsContainContact(ArrayList<Contact> contacts, RigidBody a, RigidBody b)
	{
		for (int i = 0; i < contacts.size(); i++)
		{
			if ((contacts.get(i).a() == a || contacts.get(i).a() == b) &&
					(contacts.get(i).b() == a || contacts.get(i).b() == b) )
					return true;
		}

		return false;
	}

	/** Check if two bounding Circles are colliding */
	private static boolean boxesAreColliding(RigidBody aR, RigidBody bR)
	{
		BoundingBox a = aR.bounds();
		BoundingBox b = bR.bounds();

		if ((a.leftX(aR.center()) < b.rightX(bR.center()))
				&& (a.rightX(aR.center()) > b.leftX(bR.center()))
				&& (a.topY(aR.center()) < b.bottomY(bR.center()))
				&& (a.bottomY(aR.center()) > b.topY(bR.center())))
			return true;

		return false;
	}

	/** Collision detection for circle and circle */
	private static Contact circleCircle(Circle a, Circle b)
	{
		Contact contact = null;
		float radiusA = a.bounds().radius();
		float radiusB = b.bounds().radius();
		Vector2 AB = b.center().subtractBy(a.center());
		float radiiSquare = radiusA * radiusA + radiusB * radiusB;

		// if ||A-B||^2 < (r1 + r2) ^2 then we have a circle collision
		// (ie if the distance between two circles is smaller than their radii)
		if (AB.magnitudeSquared() < radiiSquare)
		{
			// Find penetration ( = RadiusA + RadiusB - |A - B| )
			float penetration = radiusA + radiusB - AB.magnitude();

			// Find normal (normal = A - B / (magnitude (A-B) )
			Vector2 normal = AB.normalize();

			// Find contact point ( = CenterA + radiusA * normal )
			Vector2 contactPoint = normal.multiplyBy(radiusA).addTo(a.center());

			contact = new Contact(a, b, Globals.DEFAULT_RESTITUTION, penetration);
			contact.setNormal(normal);
			contact.setContactPoint(contactPoint);
		}

		return contact;
	}

	/** Collision between a circle and a plane/edge */
	private static Contact circlePolygon(Circle circle, PolyBody polygon)
	{
		Contact contact = null;
		Vector2 contactPoint = null;
		Vector2 contactNormal = null;
		float radiusC = circle.bounds().radius();
		Vector2 centerC = circle.center();
		ArrayList<Vector2> vertices = polygon.verticesWorld();

		// Now fine the closest point on the polygon's edge to the circle's center
		float minDist = Globals.INFINITY;
		for (int i = 0; i < polygon.numVertices(); i++)
		{
			Vector2 v0 = vertices.get(i);
			Vector2 v1 = vertices.get(((i + 1) % polygon.numVertices()));
			float dist = centerC.minimumDistanceToLine(v0, v1);
			if (Math.abs(dist) < minDist)
			{
				minDist = dist;

				// normal will always be the face normal of the edge
				contactNormal = polygon.normalWorld(i).normalize().invert();

				// contactPoint will be the center circle - the normal * radius
				contactPoint = centerC.subtractBy(contactNormal.multiplyBy(radiusC));

				// clamp it to the edge
				contactPoint = contactPoint.projectPointOntoEdge(v0, v1);
			}
		}

		// penetration is the difference in distance from center to contactPoint
		// and radius, if we are not penetrating we can bail early
		float penetration = (centerC.subtractBy(contactPoint).magnitudeSquared()) - (radiusC * radiusC);
		if (penetration > 0)
			return null;

		if (contactPoint != null && contactNormal != null)
		{
			contact = new Contact(circle, polygon, Globals.DEFAULT_RESTITUTION, penetration);
			contact.setNormal(contactNormal);
			contact.setContactPoint(contactPoint);
		}

		return contact;
	}

	/** Returns the contact point using the Minkowski Difference for polygon v polygon */
	private static Contact polygonPolygon(PolyBody a, PolyBody b)
	{
		Contact contact = null;
		float leastPenetratingDist = -Globals.INFINITY;
		float dist;
		Vector2 contactPoint = null;
		Vector2 contactNormal = null;

		// For face a, check all supporting vertices of B
		ArrayList<Vector2> vertices = a.verticesWorld();
		for (int i = 0; i < a.numVertices(); i++)
		{
			Vector2 normal = a.normalWorld(i);
			Vector2 v0 = vertices.get(i);
			Vector2 v1 = vertices.get(((i + 1) % a.numVertices()));

			// Gather support vertices of B, most opposite of face normal
			ArrayList<SupportVertex> supportVertices = getSupportVertices(b, normal.invert());

			for (int j = 0; j < supportVertices.size(); j++)
			{
				// form point on plane on minkowski face
				Vector2 mfp0 = supportVertices.get(j).vector.subtractBy(v0);
				Vector2 mfp1 = supportVertices.get(j).vector.subtractBy(v1);

				float faceDist = mfp0.dot(normal);

				Vector2 projection = Globals.ZERO_VECTOR.projectPointOntoEdge(mfp0, mfp1);
				dist = projection.magnitude() * Math.signum(faceDist);

				// collision found
				if (dist > leastPenetratingDist)
				{
					leastPenetratingDist = dist;
					contactNormal = normal;

					// if there are two support vertices we take the midpoint
					if (supportVertices.size() > 1)
					{
						int index0 = supportVertices.get(0).index;
						int index1 = supportVertices.get(1).index;
						contactPoint = b.verticesWorld().get(index0).midPoint(b.verticesWorld().get(index1));
					}
					else
					{
						int index = supportVertices.get(j).index;
						contactPoint = b.verticesWorld().get(index);
					}

					// clamp point to edge
					contactPoint = contactPoint.projectPointOntoEdge(v0, v1);
				}

				// no collision, return early
				else if (dist > 0)
					return null;
			}
		}

		// repeat for face of B, vertices of A
		vertices = b.verticesWorld();
		for (int i = 0; i < b.numVertices(); i++)
		{
			Vector2 normal = b.normalWorld(i);
			Vector2 v0 = vertices.get(i);
			Vector2 v1 = vertices.get(((i + 1) % b.numVertices()));		//modulo incase we go over it'll loop around

			// Gather support vertices of A, most opposite of face normal
			ArrayList<SupportVertex> supportVertices = getSupportVertices(a, normal.invert());

			for (int j = 0; j < supportVertices.size(); j++)
			{
				// form point on plane on minkowski face
				Vector2 mfp0 = v0.subtractBy(supportVertices.get(j).vector);
				Vector2 mfp1 = v1.subtractBy(supportVertices.get(j).vector);

				float faceDist = -mfp0.dot(normal);
				Vector2 projection = Globals.ZERO_VECTOR.projectPointOntoEdge(mfp0, mfp1);
				dist = projection.magnitude() * Math.signum(faceDist);

				// if collision
				if (dist > leastPenetratingDist)
				{
					leastPenetratingDist = dist;
					contactNormal = normal.invert();

					// if there are two support vertices we take the midpoint
					if (supportVertices.size() > 1)
					{
						int index0 = supportVertices.get(0).index;
						int index1 = supportVertices.get(1).index;
						contactPoint = a.verticesWorld().get(index0)
								.midPoint(a.verticesWorld().get(index1));
					} else
					{
						int index = supportVertices.get(j).index;
						contactPoint = a.verticesWorld().get(index);
					}

					// clamp contact point to the edge
					contactPoint = contactPoint.projectPointOntoEdge(v0, v1);
				}

				// separating axis, bail early
				else if (dist > 0)
					return null;
			}
		}

		if (leastPenetratingDist < 0 && contactPoint != null && contactNormal != null)
		{
			contact = new Contact(a, b, Globals.DEFAULT_RESTITUTION, -leastPenetratingDist);
			contact.setContactPoint(contactPoint);
			contact.setNormal(contactNormal.normalize());

			// System.out.println("ContactNormal: " + contactNormal +
			// "   contactPoint: " + contactPoint + "   Penetration: " +
			// leastPenetratingDist + "  a: " + a + "   b: " + b);
			// System.out.println("------------");
		}

		return contact;
	}

	/** For minkowski difference, get the supporting vertices given a normal. */
	private static ArrayList<SupportVertex> getSupportVertices(PolyBody body, Vector2 n)
	{
		ArrayList<SupportVertex> supportVertices = new ArrayList<SupportVertex>(body.polygon().npoints + 1);
		Vector2 normal = body.state().rotateIntoSpaceOf(n);

		int closestI = -1;
		int secondClosestI = -1;
		float closestD = -Globals.INFINITY;

		// first support
		for (int i = 0; i < body.numVertices(); i++)
		{
			float dist = normal.dot(body.vertices().get(i));

			if (dist > closestD)
			{
				closestD = dist;
				closestI = i;

				// clear second support
				secondClosestI = -1;
			} else if (dist == closestD)
			{
				// second support
				secondClosestI = i;
			}
		}

		supportVertices.add(new SupportVertex(body.state().TransformBy(body.vertices().get(closestI)), closestI));

		// if we have a second support only
		if (secondClosestI != -1)
			supportVertices.add(new SupportVertex(body.state().TransformBy(
					body.vertices().get(secondClosestI)), secondClosestI));

		return supportVertices;
	}
}