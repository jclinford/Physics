package com.sjsu.physics.collisiondetection;

import java.util.ArrayList;

import com.sjsu.physics.core.Contact;
import com.sjsu.physics.shapes.BoundingBox;
import com.sjsu.physics.shapes.Circle;
import com.sjsu.physics.shapes.PolyBody;
import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.shapes.RigidBody.BodyType;
import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.StateMatrix;
import com.sjsu.physics.utils.Vector2;

/* 
 * Performs fine collision detection.
 * No false positives nor false negatives.
 */
public class FineCollision 
{
	/* Check for a collision. If one is found we return a Contact, otherwise we return null */
	public static void getContactPoints(RigidBody a, RigidBody b, ArrayList<Contact> contacts)
	{
		// Early-out check to make sure boundingCircles are intersecting
		if (!boxesAreColliding(a, b))
			return;
				
		// make sure we don't already have this contact stored
		if (contactsContainContact(contacts, a, b))
			return;
		
		// if both objects are infinity then we do nothing
		if (a.inverseMass() == 0 && b.inverseMass() == 0)
			return;
		
		// check if we need to swap bodies. The lowest body id is always a
		checkForSwap(a, b);

		// Circle on circle collision
		if (a.type() == BodyType.CIRCLE && b.type() == BodyType.CIRCLE)
			circleCircle((Circle) a, (Circle) b, contacts);
		else if (a.type() == BodyType.CIRCLE && b.type() == BodyType.POLYGON)
			circlePolygon((Circle) a, (PolyBody) b, contacts);
		else if (a.type() == BodyType.POLYGON && b.type() == BodyType.CIRCLE)
			circlePolygon((Circle) b, (PolyBody) a, contacts);
		else if (a.type() == BodyType.POLYGON && b.type() == BodyType.POLYGON)
		{
			Contact c = null;
			c = polygonPolygon((PolyBody) a, (PolyBody) b);
			if (c != null)
				contacts.add(c);
		}
		else
		{
			System.out.println("Warning: Uknown collision type!");
		}

		return;
	}
	
	private static void checkForSwap(RigidBody a, RigidBody b)
	{
		RigidBody tmpBody;
		
		if (b.id() < a.id())
		{
			tmpBody = a;
			a = b;
			b = tmpBody;
		}
	}
	
	private static boolean contactsContainContact(ArrayList<Contact> contacts, RigidBody a, RigidBody b)
	{
		for (int i = 0; i < contacts.size(); i++)
		{
			if (contacts.get(i).a().equals(a) || contacts.get(i).a().equals(b))
				if (contacts.get(i).b().equals(a) || contacts.get(i).b().equals(b))
					return true;
		}
		return false;
	}
	
	
	/* Check if two bounding Circles are colliding */
	private static boolean boxesAreColliding(RigidBody aR, RigidBody bR)
	{
		BoundingBox a = aR.bounds();
		BoundingBox b = bR.bounds();
		
		if ( (a.leftX(aR.center()) < b.rightX(bR.center())) &&
				(a.rightX(aR.center()) > b.leftX(bR.center())) &&
				(a.topY(aR.center()) < b.bottomY(bR.center())) &&
				(a.bottomY(aR.center()) > b.topY(bR.center())) )
			return true;
		
		return false;
	}


	/* Collision detection for circle and circle */
	private static void circleCircle(Circle a, Circle b, ArrayList<Contact> contacts)
	{
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

			Contact contact = new Contact(a, b, Globals.DEFAULT_RESTITUTION, penetration);
			contact.setNormal(normal);
			contact.setContactPoint(contactPoint);
			contacts.add(contact);
		}
	}

	/* Collision between a circle and a plane/edge */
	private static void circlePolygon(Circle circle, PolyBody polygon, ArrayList<Contact> contacts)
	{
		float penetration = 0;
		Vector2 contactPoint = null;
		Vector2 contactNormal = null;
		
		float radiusC = circle.bounds().radius();
		Vector2 centerC = circle.center();
		ArrayList<Vector2> vertices = polygon.verticesWorld();
		
		// Now fine the closest point on the polygon's edge to the circles center
		float minDist = Globals.INFINITY;
		for (int i = 0; i < polygon.numVertices(); i++)
		{
			Vector2 v0 = vertices.get(i);
			Vector2 v1 = vertices.get( ((i + 1) % polygon.numVertices()) );
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
		
		// penetration is the difference in distance from center to contactPoint and radius
		penetration = (centerC.subtractBy(contactPoint).magnitudeSquared()) - (radiusC * radiusC);
		
		// if we are not penetrating we can bail early
		if (penetration > 0)
			return;

		if (contactPoint == null || contactNormal == null)
		{
			System.out.println("No contact found");
			return;
		}

		Contact contact = new Contact(circle, polygon, Globals.DEFAULT_RESTITUTION, penetration);
		contact.setNormal(contactNormal);
		contact.setContactPoint(contactPoint);
		contacts.add(contact);
		return;
	}
	
	
	/* Returns the contact point using the Minkowski Difference for polygon v polygon */
	private static Contact polygonPolygon(PolyBody a, PolyBody b)
	{
		float leastPenetratingDist = -Globals.INFINITY;
		float dist;
		Vector2 contactPoint = null;
		Vector2 contactNormal = null;
		
		// For face a, check all supporting vertexs of B
		ArrayList<Vector2> vertices = a.verticesWorld();
		for (int i = 0; i < a.numVertices(); i++)
		{
			// get the normal
			Vector2 normal = a.normalWorld(i);
						
			// gather A's edge
			Vector2 v0 = vertices.get(i);
			Vector2 v1 = vertices.get( ((i + 1) % a.numVertices()) );
						
			// Gather support vertices of B, most opposite of face normal
			ArrayList<SupportVertex> supportVertices = getSupportVertices(b, normal.invert());
			
			for (int j = 0; j < supportVertices.size(); j++)
			{
				// form point on plane on minkowski face
				Vector2 mfp0 = supportVertices.get(j).vector.subtractBy(v0);
				Vector2 mfp1 = supportVertices.get(j).vector.subtractBy(v1);
				
				float faceDist = mfp0.dot(normal);

				// project onto minkowski edge
				Vector2 projection = Globals.ZERO_VECTOR.projectPointOntoEdge(mfp0, mfp1);

				// Get distance
				dist = projection.magnitude() * Math.signum(faceDist);

				// track neg
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
				
				// track pos
				if (dist > 0)
				{
					return null;
				}
			}
		}

		// repeat for face of B, vertices of A
		vertices = b.verticesWorld();
		for (int i = 0; i < b.numVertices(); i++)
		{
			// get the normal
			Vector2 normal = b.normalWorld(i);
			
			// gather b's edge
			Vector2 v0 = vertices.get(i);
			Vector2 v1 = vertices.get( ((i + 1) % b.numVertices()) );	// modulu in case we go over numVertices to loop back around

			// Gather support vertices of A, most opposite of face normal
			ArrayList<SupportVertex> supportVertices = getSupportVertices(a, normal.invert());
			
			for (int j = 0; j < supportVertices.size(); j++)
			{
				// form point on plane on minkowski face
				Vector2 mfp0 = v0.subtractBy(supportVertices.get(j).vector);
				Vector2 mfp1 = v1.subtractBy(supportVertices.get(j).vector);

				float faceDist = -mfp0.dot(normal);

				// project onto minkowski edge
				Vector2 projection = Globals.ZERO_VECTOR.projectPointOntoEdge(mfp0, mfp1);

				// Get distance
				dist = projection.magnitude() * Math.signum(faceDist);

				// track neg
				if (dist > leastPenetratingDist)
				{
					leastPenetratingDist = dist;
					contactNormal = normal.invert();
					
					// if there are two support vertices we take the midpoint
					if (supportVertices.size() > 1)
					{
						int index0 = supportVertices.get(0).index;
						int index1 = supportVertices.get(1).index;
						contactPoint = a.verticesWorld().get(index0).midPoint(a.verticesWorld().get(index1));
					}
					else
					{
						int index = supportVertices.get(j).index;
						contactPoint = a.verticesWorld().get(index);
					}
					
					// clamp contact point to the edge
					contactPoint = contactPoint.projectPointOntoEdge(v0, v1);
				}

				// if we get a positive we can bail early, this means there is a separating axis
				if (dist > 0)
				{
					return null;
				}
			}
		}
		
		if (leastPenetratingDist < 0 && contactPoint != null && contactNormal != null)
		{
			Contact contact = new Contact(a, b, Globals.DEFAULT_RESTITUTION, -leastPenetratingDist);
			
			contact.setContactPoint(contactPoint);
			contact.setNormal(contactNormal.normalize());
			
//			System.out.println("ContactNormal: " + contactNormal + "   contactPoint: " + contactPoint + "   Penetration: " + 
//					leastPenetratingDist + "  a: " + a + "   b: " + b);
//			System.out.println("------------");
			
			return contact;
		}
		
		return null;
	}
	
	
	private static ArrayList<SupportVertex> getSupportVertices(PolyBody body, Vector2 n)
	{
		ArrayList<SupportVertex> supportVertices = new ArrayList<SupportVertex>(body.polygon().npoints + 1);
		
		// rotate into polygon space
		Vector2 normal = body.state().rotateIntoSpaceOf(n);
		
		// get axis bits
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
			}
			else if (dist == closestD)
			{
				// second support
				secondClosestI = i;
			}
		}
		
		supportVertices.add(new SupportVertex(body.state().TransformBy(body.vertices().get(closestI)), closestI));
		if (secondClosestI != -1)
			supportVertices.add(new SupportVertex(body.state().TransformBy(body.vertices().get(secondClosestI)), secondClosestI));
		return supportVertices;
	}
}	
	
	
	
	
	
	
	
	
	
	
	
	
//	/* Check if polygon a and b collide using angular casting
//	 * Returns the time they intersect, or -1 if there is no intersection */
//	public static Contact polygonPolygon(PolyBody a, PolyBody b)
//	{
//		// find if there is an initial contact
//		Contact contact = polygonPolygonContact(a, b);
//		if (contact == null)
//			return null;
//		
//		int iterations = 0;		
//		float t = 0;
//		
//		// back these up
//		StateMatrix realMa = new StateMatrix(a.center(), a.orientation());
//		StateMatrix realMb = new StateMatrix(b.center(), b.orientation());
//		
//		// relative linear velocity
//		Vector2 relVel = b.velocity().subtractBy(a.velocity());
//		
//		StateMatrix mA = new StateMatrix(a.center().addTo(a.velocity().multiplyBy(t)), 
//				a.orientation() + (a.angularVelocity() * t));
//		StateMatrix mB = new StateMatrix(b.center().addTo(b.velocity().multiplyBy(t)),
//				b.orientation() + (b.angularVelocity() * t));
//		
//		// contact already generated from polygonPolygon using minkowski
//		while (contact.penetration() > Globals.ANGULAR_TOLLERANCE && 
//				iterations < Globals.MAX_ANGULAR_CAST_ITERATIONS)
//		{
//			// intersect velocity against normal
//			float rN = relVel.dot(contact.normal());
//			float distRel = rN + a.bounds().radius() * Math.abs(a.angularVelocity()) +
//					b.bounds().radius() * Math.abs(b.angularVelocity());
//			
//			// computer conservative advancement
//			t += contact.penetration() / distRel;
//			
//			if (t < 0 || t > 1)
//			{
//				// does not connect, restore matrixes
//				a.state().set(realMa);
//				b.state().set(realMb);
//				contact = null;
//				return null;
//			}
//			
//			// interpolate 
//			mA = new StateMatrix(a.center().addTo(a.velocity().multiplyBy(t)), 
//					a.orientation() + (a.angularVelocity() * t));
//			mB = new StateMatrix(b.center().addTo(b.velocity().multiplyBy(t)),
//					b.orientation() + (b.angularVelocity() * t));
//			a.state().set(mA);
//			b.state().set(mB);
//			
//			// get new distance
//			contact = polygonPolygon(a, b);
//			iterations++;
//		}
//		
//		// restore matrixes and return time
//		a.state().set(realMa);
//		b.state().set(realMb);
//		return contact;
//	}
//	
	
	
	
	
	

//	/* Finds the contact point assuming edge vs point
//	 * Does this by comparing distance of vertex's to center. The vertice that is closest
//	 * to the center should be the contact point. If two vertices are equal distance away then 
//	 * we have an edge/edge collision, and the midpoint of these two vertices will be returned */
//	private static void polygonPolygon(RigidBody aR, RigidBody bR, ArrayList<Contact> contacts)
//	{
//		PolyBody a = (PolyBody) aR;
//		PolyBody b = (PolyBody) bR;
//		Vector2 contactPoint1 = null;
//		Vector2 contactPoint2 = null;
//		Vector2 normal1 = null;
//		Vector2 normal2 = null;
//		Vector2 minDistance1 = new Vector2(10, 10);
//		Vector2 minDistance2 = new Vector2(10, 10);
//
//
//		// We check if any vertex of a is contained in b
//		ArrayList<Vector2> vertices = a.verticesWorld();
//		for (int i = 0; i < a.numVertices(); i++)
//		{
//			for (int j = 0; j < b.numVertices(); j++)
//			{
//				// edge V1-V0
//				Vector2 v1 = b.verticesWorld().get( (j + 1) % b.numVertices());
//				Vector2 v2 = b.verticesWorld().get(j);
//				
//				// distance vertex A is to V1-V0
//				Vector2 dist = vertices.get(i).projectPointOntoEdge(v1, v2);
//				dist.subtract(vertices.get(i));
////				System.out.println("Penetration: " + dist + "   eV" + (j + 1)%a.numVertices() + ":" + v1 + "  eV" + j + ":" + v2 + "   v:" + vertices.get(i));
//
//				if (dist.magnitudeSquared() < minDistance1.magnitudeSquared())
//				{
//					minDistance1 = dist;
//					contactPoint1 = vertices.get(i).copy();
//					Vector2 norm = v1.copy();
//					norm.subtract(v2);
//					normal1 = norm;
//					
//					// clear second support
//					contactPoint2 = null;
//					normal2 = null;
//					minDistance2 = null;
//				}
//				else if (dist.magnitudeSquared() == minDistance1.magnitudeSquared())
//				{
//					// second support found
//					minDistance2 = dist;
//					contactPoint2 = vertices.get(i).copy();
//					Vector2 norm = v1.copy();
//					norm.subtract(v2);
//					normal2 = norm;
//				}
//			}
//		}
//
//		// now check for vertex's of b against a if we didn't find a negative value above
//		if (minDistance1.magnitudeSquared() > 0)
//		{
//			vertices = b.verticesWorld();
//			for (int i = 0; i < b.numVertices(); i++)
//			{
//				for (int j = 0; j < a.numVertices(); j++)
//				{
//					// edge V1-V0
//					Vector2 v1 = a.verticesWorld().get( (j + 1) % a.numVertices()).copy();
//					Vector2 v2 = a.verticesWorld().get(j);
//
//					// distance vertex A is to V1-V0
//					Vector2 dist = vertices.get(i).projectPointOntoEdge(v1, v2);
//					dist.subtract(vertices.get(i));
////					System.out.println("Penetration: " + dist + "   eV" + (j + 1)%a.numVertices() + ":" + v1 + "  eV" + j + ":" + v2 + "   v:" + vertices.get(i));
//
//					if (dist.magnitudeSquared() < minDistance1.magnitudeSquared())
//					{
//						minDistance1 = dist;
//						contactPoint1 = vertices.get(i).copy();
//						Vector2 norm = v1.copy();
//						norm.subtract(v2);
//						normal1 = norm;
//						
//						// clear second support
//						contactPoint2 = null;
//						normal2 = null;
//						minDistance2 = null;
//					}
//					else if (dist.magnitudeSquared() == minDistance1.magnitudeSquared())
//					{
//						// second support found
//						minDistance2 = dist;
//						contactPoint2 = vertices.get(i).copy();
//						Vector2 norm = v1.copy();
//						norm.subtract(v2);
//						normal2 = norm;
//					}
//				}
//			}
//		}
//		
//		if (normal1 == null || contactPoint1 == null)
//			return;
//		
//		Contact contact1 = new Contact(a, b, Globals.DEFAULT_RESTITUTION, minDistance1.magnitude());
//		contact1.setContactPoint(contactPoint1);
//		normal1.rotateThis90();		// this normal is just V1-V2, need to rotate and normalize to get actual normal
//		normal1.normalize();
//		contact1.setNormal(normal1);
//		contacts.add(contact1);
////		System.out.println(" a: " + a + "     b: " + b);
////		System.out.println("Contact1 : " + contactPoint1 + "   pen" + minDistance1);
//
//		
//		// if we have a second contact (edge/edge collision) then we need to add it to the list
//		if (contactPoint2 != null && normal2 != null && minDistance2 != null)
//		{
//			Contact contact2 = new Contact(a, b, Globals.DEFAULT_RESTITUTION, minDistance2.magnitude());
//			contact2.setContactPoint(contactPoint2);
//			normal2.rotateThis90();
//			normal2.normalize();
//			contact2.setNormal(normal2);
//			contacts.add(contact2);
//			
////			System.out.println("Contact2 : " + contactPoint2 + "   pen" + minDistance2);
//		}
//			
//		return;
//	}

	
//	/* Uses SAT to find contact points */
//	private static void polygonPolygon(RigidBody aR, RigidBody bR)
//	{
//		//		System.out.println("Polypoly collision");
//
//		PolyBody a = (PolyBody) aR;
//		PolyBody b = (PolyBody) bR;
//		ArrayList<Vector2> axes = new ArrayList<Vector2>(32);	// holds all the normals we calculate
//
//		ArrayList<Vector2> vertices = a.vertices();
//		int j = vertices.size() - 1;
//		for (int i = 0; i < vertices.size(); i++)
//		{
//			// Calculate the normal for each edge of A
//			Vector2 e = vertices.get(i).copy();
//			e.subtract(vertices.get(j));
//			Vector2 normal = new Vector2(-e.y(), e.x());
//
//			// see if we can draw a line through the polygons
//			if (axisSeparatePolygons(normal, a, b))
//				return null;
//
//			axes.add(normal);
//			j = i;
//		}
//
//		// Do the same thing for vertices of B
//		vertices = b.vertices();
//		j = vertices.size() - 1;
//		for (int i = 0; i < vertices.size(); i++)
//		{
//			// Calculate the normal for each edge of A
//			Vector2 e = vertices.get(i).copy();
//			e.subtract(vertices.get(j));
//			Vector2 normal = new Vector2(-e.y(), e.x());
//
//			// see if we can draw a line through the polygons
//			if (axisSeparatePolygons(normal, a, b))
//				return null;
//
//			axes.add(normal);
//			j = i;
//		}
//
//		// If we tried all possible normals and got here, then we are colliding
//
//		// find the normal and the penetration (the magnitude of this normal)
//		Vector2 normal = findSATPenetration(axes);
//		Vector2 AB = a.center().copy();
//		AB.subtract(b.center());
//
//		if (AB.dot(normal) < 0.0f)
//			normal.invert();
//
//		/* Resolve penetration ourselves */
//		normal.multiply(.5f);
//		Vector2 cenA = a.center().copy();
//		cenA.add(normal);
//		a.setCenter(cenA);
//
//		Vector2 cenB = b.center().copy();
//		cenB.subtract(normal);
//		b.setCenter(cenB);
//		//		float penetration = normal.magnitude();
//		//		normal.normalize();
//
//		// penetration is 0, already resolved it..
//		Contact contact = new Contact(a, b, Globals.DEFAULT_RESTITUTION, 0);
//		contact.setNormal(normal);
////		contact.setContactPoint(edgePointFindContactPoint(a, b));
//		return contact;
//	}

//	/* Attempt to draw a line through two polygons to check for intersection */
//	private static boolean axisSeparatePolygons(Vector2 axis, PolyBody a, PolyBody b)
//	{
//		Vector2 minMaxA, minMaxB;			// 2d vector with <minValue, maxValue>
//
//		minMaxA = calculateInterval(axis, a);
//		minMaxB = calculateInterval(axis, b);
//
//		float minA = minMaxA.x();
//		float maxA = minMaxA.y();
//		float minB = minMaxB.x();
//		float maxB = minMaxB.y();
//
//		if (minA > maxB || minB > maxA)
//			return true;
//
//		return false;
//	}
//
//	/* Used in SAT */
//	private static Vector2 calculateInterval(Vector2 axis, PolyBody p)
//	{
//		float min = axis.dot(p.vertices().get(0));
//		float max = min;
//
//		for (int i = 0; i < p.numVertices(); i++)
//		{
//			float d = p.vertices().get(i).dot(axis);
//			if (d < min)
//				min = d;
//			else if (d > max)
//				max = d;
//		}
//
//		return new Vector2(min, max);
//	}
//
//	/* Returns the penetration (MTD in SAT) of the axes we discovered in SAT */
//	private static Vector2 findSATPenetration(ArrayList<Vector2> axes)
//	{
//		Vector2 penetration = axes.get(0);
//		float minDistance = axes.get(0).dot(axes.get(0));
//
//		for (int i = 1; i < axes.size(); i++)
//		{
//			//			System.out.println("Axes: " + axes.get(i));
//			float distance = axes.get(i).dot(axes.get(i));
//			if (distance < minDistance)
//			{
//				minDistance = distance;
//				penetration = axes.get(i);
//			}
//		}
//		return penetration;
//	}