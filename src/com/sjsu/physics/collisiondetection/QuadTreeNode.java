package com.sjsu.physics.collisiondetection;

import java.awt.Rectangle;
import java.util.ArrayList;

import com.sjsu.physics.shapes.PolyBody;
import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.shapes.RigidBody.BodyType;
import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;


/** 
 * A node of a quad-tree, built for parallel processing.
 * Similar to a Binary tree, but with four children.
 * 
 * 
 * If there is overlap, the object will be placed with the parent node's borderBodies.
 *  So if an object is overlapping two processor's boundaries, it will be placed
 * in the worldRoot node. Each processor can then check for collisions solely 
 * within it's own children and also against the worldRootNode's children.
 * 
 * 
 * 							  World (All bodies)
 * 									  |
 * 			Process1		Process2	Process3		Process4
 * 				|				|			|				|				
 * 			Children		Children	Children		Children
 */

public class QuadTreeNode
{
	private Rectangle bounds;
	private int depth;
	private boolean isLeaf;

	private ArrayList<RigidBody> myBodies; // Bodies that belong to this node
	private QuadTreeNode parent;
	private QuadTreeNode[] children; // Children nodes of this node

	public QuadTreeNode()
	{
		// Use init method for recursive initiation
	}

	/* Recursively init the quadtree */
	public QuadTreeNode init(QuadTreeNode par, Rectangle grid, int d)
	{
		bounds = grid;
		depth = d;
		isLeaf = false;

		myBodies = new ArrayList<RigidBody>(Globals.MAX_QUADTREE_CHILDREN);
		children = new QuadTreeNode[4];
		parent = par;

		if (Globals.MAX_QUADTREE_DEPTH > depth)
			this.subDivide();
		else
			isLeaf = true;

		return this;
	}

	/* Subdivide a node into four subnodes */
	private void subDivide()
	{
		if (depth > Globals.MAX_QUADTREE_DEPTH)
			return;

		int bX = bounds.x;
		int bY = bounds.y;

		int bwh = (int) Math.floor((bounds.width / 2));
		int bhh = (int) Math.floor((bounds.height / 2));
		int bXbwh = bX + bwh;
		int bYbhh = bY + bhh;

		/* Rect's for new node bounds */
		Rectangle topLeftBounds = new Rectangle(bX, bY, bwh, bhh);
		Rectangle topRightBounds = new Rectangle(bXbwh, bY, bwh, bhh);
		Rectangle bottomLeftBounds = new Rectangle(bX, bYbhh, bwh, bhh);
		Rectangle bottomRightBounds = new Rectangle(bXbwh, bYbhh, bwh, bhh);

		/* Create the new Nodes */
		children[Globals.TOP_LEFT] = new QuadTreeNode().init(this, topLeftBounds, depth + 1);
		children[Globals.TOP_RIGHT] = new QuadTreeNode().init(this, topRightBounds, depth + 1);
		children[Globals.BOTTOM_RIGHT] = new QuadTreeNode()
				.init(this, bottomRightBounds, depth + 1);
		children[Globals.BOTTOM_LEFT] = new QuadTreeNode().init(this, bottomLeftBounds, depth + 1);
	}

	/*
	 * Test whether a rigidBody is fully contained inside this node's bounds if
	 * the node can fully contain the body then it returns true, else false TODO
	 * can I be more efficient here??
	 */
	public boolean contains(RigidBody body)
	{
		// check used for circles
		if (body.type() == BodyType.CIRCLE)
		{
			Vector2 center = body.center();
			float radius = body.bounds().radius();

			if (bounds.getMaxX() > (center.x() + radius)
					&& bounds.getMinX() < (center.x() - radius)
					&& bounds.getMaxY() > (center.y() + radius)
					&& bounds.getMinY() < (center.y() - radius))
				return true;
			return false;
		}

		// check used for polygons
		if (body.type() == BodyType.POLYGON)
		{
			// if any vertex is outside of the bounds then we are not contained
			PolyBody poly = (PolyBody) body;
			ArrayList<Vector2> vertices = poly.verticesWorld();

			for (int i = 0; i < vertices.size(); i++)
			{
				if (!bounds.contains(vertices.get(i).x(), vertices.get(i).y()))
					return false;
			}
			return true;
		}

		return false;
	}

	/* Insert a body to our node. If it's full split our node */
	public void insert(RigidBody body)
	{
		// If we have subnodes, find the index our body belongs to and insert it there
		if (!isLeaf)
		{
			// Find which quadrant the center is in
			int index = findIndex(body);

			// Make sure the quadrant can fully contain the body's geometry
			if (children[index].contains(body))
			{
				children[index].insert(body);
				body.depth = depth + 1;
				return;
			}
			// if child can't contain it, see if this node can
			else if (this.contains(body))
			{
				myBodies.add(body);
				body.depth = depth;
				return;
			}
			// if child and this cannot contain, try parent
			else if (parent != null)
			{
				if (parent.contains(body))
				{
					parent.bodies().add(body);
					body.depth = depth - 1;
					return;
				}
			}
		}

		// if we are at the leaf node then just add it here
		myBodies.add(body);
		body.depth = depth;
		return;
	}

	/** Finds the index that a body belongs to based on center point */
	private int findIndex(RigidBody body)
	{
		int index;
		boolean left = (body.center().x() > (bounds.x + bounds.width / 2)) ? false : true;
		boolean top = (body.center().y() > (bounds.y + bounds.height / 2)) ? false : true;

		if (left)
		{
			if (top)
				index = Globals.TOP_LEFT;
			else
				index = Globals.BOTTOM_LEFT;
		} else
		{
			if (top)
				index = Globals.TOP_RIGHT;
			else
				index = Globals.BOTTOM_RIGHT;
		}

		return index;
	}

	/**
	 * Clear all bodies this this node and subnodes.. does not unallocate the
	 * node memory nor does it unallocate or mess with the init'd tree structure
	 */
	public void clearObjects()
	{
		myBodies.clear();

		for (int i = 0; i < 4; i++)
		{
			if (children[i] != null)
				children[i].clearObjects();
		}
	}
	
	
	/**
	 * Retrieve all the bodies that could be colliding with this body.
	 * (the node that the body resides in and all parent nodes)
	 */
	public ArrayList<RigidBody> getPossibleCollisions(RigidBody body)
	{
		// Check if this node contains the body
		if (myBodies.contains(body))
		{
			/* if it does contain, the possible collisions
			are this node and all parents to this node */
			ArrayList<RigidBody> collisions = new ArrayList<RigidBody>(myBodies.size() * 2);
			collisions.addAll(myBodies);
			
			/* add all parents bodies to list */
			QuadTreeNode par = this.parent;
			while (par != null && par.depth > 0)
			{
				collisions.addAll(par.bodies());
				par = par.parent;
			}
			
			return collisions;
		}
		// if we do not contain and we are depth 1, check the worldRoot node first (depth 0)
		else if (depth == 1 && parent != null && parent.bodies().contains(body))
		{
			return parent.bodies();
		}
		// otherwise look deeper into the tree
		else if (!isLeaf)
		{
			int index = findIndex(body);
			return children[index].getPossibleCollisions(body);
		}

		// if we are at the base node already and we do not contain
		// then return an empty list
		return new ArrayList<RigidBody>();
	}
	
	
	/**
	 * Removes the body from the quadtree
	 * @param body The body to be removed
	 * @return
	 */
	public void removeBody(RigidBody body)
	{
		getNode(body).bodies().remove(body);
	}

	/** 
	 * Retrieve the node that the body belongs to
	 * @param body Body to search for
	 * @return	Node that body belongs to
	 */
	public QuadTreeNode getNode(RigidBody body)
	{
		// If we have subnodes, find the index
		if (!isLeaf)
		{
			// if this node contains we return
			if (myBodies.contains(body))
				return this;
			
			// otherwise search deeper in the tree
			int index = findIndex(body);
			return children[index].getNode(body);
		}

		// if we are at the leaf node return
		return this;
	}
	
	
	/** 
	 * Retrieve the depth that the body is at
	 * @param body Body to search for
	 * @return	Depth of the body in the quadtree
	 */
	public int getDepth(RigidBody body)
	{
		// If we have subnodes, find the index
		if (!isLeaf)
		{
			// if this node contains we return
			if (myBodies.contains(body))
				return depth;
			
			// otherwise search deeper in the tree
			int index = findIndex(body);
			return children[index].getDepth(body);
		}

		// if we are at the leaf node return
		return depth;
	}

	/** Return our children / subnodes */
	public QuadTreeNode[] children()
	{
		return children;
	}

	/** Return the bounds that this Node occupies */
	public Rectangle bounds()
	{
		return bounds;
	}

	/**
	 * Return only myBodies (objects that reside fully in the bounds of the
	 * node)
	 */
	public ArrayList<RigidBody> bodies()
	{
		return myBodies;
	}
}
