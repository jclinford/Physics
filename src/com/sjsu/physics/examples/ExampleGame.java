package com.sjsu.physics.examples;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JFrame;
import javax.swing.JPanel;

import com.sjsu.physics.collisiondetection.QuadTreeNode;
import com.sjsu.physics.core.Contact;
import com.sjsu.physics.core.World;
import com.sjsu.physics.shapes.Circle;
import com.sjsu.physics.shapes.PolyBody;
import com.sjsu.physics.shapes.RigidBody;
import com.sjsu.physics.shapes.RigidBody.BodyType;
import com.sjsu.physics.utils.Globals;
import com.sjsu.physics.utils.Vector2;

public class ExampleGame 
{
	private JFrame mainFrame;
	private static JPanel drawingPanel;

	private static final int BOX_LENGTH = 20;
	private static final int NUM_BODIES = 1000;
	private static final int MAX_VELOCITY = 35;
	private static final int MAX_MASS = 100;
	private Random generator;

	public static World world;


	public ExampleGame()
	{
		world = new World();

		mainFrame = new JFrame("Simple Phys");
		drawingPanel = new DrawingPanel();
		drawingPanel.setPreferredSize(new Dimension(Globals.MAX_GAME_WIDTH, Globals.MAX_GAME_HEIGHT));
		drawingPanel.addMouseListener(new ClickListener());
		mainFrame.setContentPane(drawingPanel);
		mainFrame.pack();
		mainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		mainFrame.setVisible(true);

		generator = new Random(System.currentTimeMillis());

		//makeWalls();
		//makeBox();
		//makeFixedBox();
		//makeRandomBodies();
		//makeHeadOn();
		//makeOffsetHeadOn();
		//makePolygons();
		//makeHeadOnPolygons();
		makeFloor();
		//makeNewtonCradle();
	}


	public static void main(String arg[])
	{	
		// create the window
		ExampleGame game = new ExampleGame();


		world.startThreads();

		// set timer to create constant callback to redraw the world
		Timer timer = new Timer("constantStep");
		PhysicsStep step = new PhysicsStep(world, drawingPanel);
		timer.schedule(step, 0, 1);
	}


	/* Make walls to constrain everything inside the box */
	/*	public void makeWalls()
	{
		Wall leftWall = new Wall();
		Wall topWall = new Wall();
		Wall rightWall = new Wall();
		Wall bottomWall = new Wall();

		leftWall.setGeometry(new Rectangle2D.Float(2, 2, 1, Globals.MAX_GAME_WIDTH - 3));
		rightWall.setGeometry(new Rectangle2D.Float(1, Globals.MAX_GAME_WIDTH-3, 1, Globals.MAX_GAME_WIDTH - 3));
		topWall.setGeometry(new Rectangle2D.Float(2, 2, Globals.MAX_GAME_WIDTH - 3, 1));
		bottomWall.setGeometry(new Rectangle2D.Float(Globals.MAX_GAME_WIDTH - 3, 2, Globals.MAX_GAME_WIDTH - 4, 1));


		world.addBodyToWorld(leftWall);
		world.addBodyToWorld(rightWall);
		world.addBodyToWorld(topWall);
		world.addBodyToWorld(bottomWall);
	}
	 */

	/* create a box of bodies to act as a wall-bounding box fixed */
	public void makeFixedBox()
	{
		int centerX = Globals.MAX_GAME_WIDTH / 2;
		int centerY = Globals.MAX_GAME_WIDTH / 2;
		Vector2 zero = new Vector2(0, 0);

		// horizontal walls
		for (int i = 0; i < BOX_LENGTH * 4 + 1; i++)
		{	
			int xLoc = (int) ((centerX - BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS * 4) + (i * Globals.DEFAULT_CIRCLE_RADIUS * 2)); 
			int yLoc1 = (int) (centerY - BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS * 4);
			int yLoc2 = (int) (centerY + BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS * 4);
			Vector2 loc1 = new Vector2(xLoc, yLoc1);
			Vector2 loc2 = new Vector2(xLoc, yLoc2);
			Circle b1 = new Circle(loc1, Globals.DEFAULT_CIRCLE_RADIUS);
			Circle b2 = new Circle(loc2, Globals.DEFAULT_CIRCLE_RADIUS);
			b1.setMass(Globals.INFINITY);
			b2.setMass(Globals.INFINITY);

			world.addBodyToWorld(b1);
			world.addBodyToWorld(b2);
		}

		// vertical walls
		for (int i = 0; i < BOX_LENGTH * 4; i++)
		{
			int yLoc = (int) ((centerY - BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS * 4) + (i * Globals.DEFAULT_CIRCLE_RADIUS) * 2); 
			int xLoc1 = (int) (centerY - BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS * 4);
			int xLoc2 = (int) (centerY + BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS * 4);
			Vector2 loc1 = new Vector2(xLoc1, yLoc);
			Vector2 loc2 = new Vector2(xLoc2, yLoc);
			Circle b1 = new Circle(loc1, Globals.DEFAULT_CIRCLE_RADIUS);
			Circle b2 = new Circle(loc2, Globals.DEFAULT_CIRCLE_RADIUS);
			b1.setMass(Globals.INFINITY);
			b2.setMass(Globals.INFINITY);


			world.addBodyToWorld(b1);
			world.addBodyToWorld(b2);
		}
	}

	/* create a box of bodies to act as a wall-bounding box */
	public void makeBox()
	{
		int centerX = Globals.MAX_GAME_WIDTH / 2;
		int centerY = Globals.MAX_GAME_WIDTH / 2;
		Vector2 zero = new Vector2(0, 0);

		// horizontal walls
		for (int i = 0; i < BOX_LENGTH + 1; i++)
		{
			int xLoc = (int) ((centerX - BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS) + (i * Globals.DEFAULT_CIRCLE_RADIUS * 2)); 
			int yLoc1 = (int) (centerY - BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS);
			int yLoc2 = (int) (centerY + BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS);

			Circle b1 = new Circle(xLoc, yLoc1, Globals.DEFAULT_CIRCLE_RADIUS);
			Circle b2 = new Circle(xLoc, yLoc2, Globals.DEFAULT_CIRCLE_RADIUS);
			//b1.setMass(Constants.INFINITY);
			//b2.setMass(Constants.INFINITY);
			b1.setMass(MAX_MASS);
			b2.setMass(MAX_MASS);

			world.addBodyToWorld(b1);
			world.addBodyToWorld(b2);
		}

		// vertical walls
		for (int i = 0; i < BOX_LENGTH; i++)
		{
			int yLoc = (int) ((centerY - BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS) + (i * Globals.DEFAULT_CIRCLE_RADIUS * 2)); 
			int xLoc1 = (int) (centerY - BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS);
			int xLoc2 = (int) (centerY + BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS);

			Circle b1 = new Circle(xLoc1, yLoc, Globals.DEFAULT_CIRCLE_RADIUS);
			Circle b2 = new Circle(xLoc2, yLoc, Globals.DEFAULT_CIRCLE_RADIUS);
			//b1.setMass(Constants.INFINITY);
			//b2.setMass(Constants.INFINITY);
			b1.setMass(20);
			b2.setMass(20);

			world.addBodyToWorld(b1);
			world.addBodyToWorld(b2);
		}
	}

	/* Make some polygons */
	public void makePolygons()
	{
		int[] xpoints = {-5, -5, 5, 5};
		int[] ypoints = {-5, 5, 5, -5};

		Polygon p = new Polygon(xpoints, ypoints, 4);

		PolyBody b1 = new PolyBody(p, 100, 200);
		PolyBody b2 = new PolyBody(p, 200, 300);

		b1.setVelocity(new Vector2(45, 0));
		b2.setVelocity(new Vector2(30, 10));

		b1.setMass(10);
		b2.setMass(20);

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);
	}

	/* 2d headon polygon collision */
	public void makeHeadOnPolygons()
	{
		int[] xpoints1 = {0, 10, 0, -10};
		int[] ypoints1 = {-10, 0, 10, 0};
		int[] xpoints2 = {-11, 11, 11, -11};
		int[] ypoints2 = {-11, -11, 11, 11};

		Polygon p1 = new Polygon(xpoints1, ypoints1, 4);
		Polygon p2 = new Polygon(xpoints2, ypoints2, 4);

		PolyBody b1 = new PolyBody(p1, 200, 630);
		PolyBody b2 = new PolyBody(p2, 500, 630);

		b1.setVelocity(new Vector2(0, 0));
		b2.setVelocity(new Vector2(-50, 0));

		b1.setMass(40);
		b2.setMass(10);

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);
	}

	/* a floor polygon */
	public void makeFloor()
	{
		int[] xpoints1 = {-200, 200, 200, -200};
		int[] ypoints1 = {-10, -10, 10, 10};
		int[] xpoints2 = {-10, 10, 10, -10};
		int[] ypoints2 = {-50, -50, 50, 50};
		int[] xpoints3 = {-10, 10, 10, -10};
		int[] ypoints3 = {-200, -200, 200, 200};

		Polygon p1 = new Polygon(xpoints1, ypoints1, 4);
		Polygon p2 = new Polygon(xpoints2, ypoints2, 4);
		Polygon p3 = new Polygon(xpoints3, ypoints3, 4);

		PolyBody b1 = new PolyBody(p1, 300, Globals.MAX_GAME_HEIGHT - 400);
		PolyBody b2 = new PolyBody(p2, 600, Globals.MAX_GAME_HEIGHT - 400);
		PolyBody b3 = new PolyBody(p3, 550, Globals.MAX_GAME_HEIGHT - 700);

		b1.setMass(Globals.INFINITY);
		b2.setMass(Globals.INFINITY);
		b3.setMass(Globals.INFINITY);

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);
		world.addBodyToWorld(b3);
	}

	/* 2D headon collision */
	public void makeOffsetHeadOn()
	{
		// // Head on collision
		Circle b1 = new Circle(new Vector2(45, Globals.MAX_GAME_WIDTH / 2 + 2), Globals.DEFAULT_CIRCLE_RADIUS);
		b1.setVelocity(new Vector2(MAX_VELOCITY, 0));
		b1.setMass(5);

		Circle b2 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH - 45, Globals.MAX_GAME_WIDTH / 2 - 8), Globals.DEFAULT_CIRCLE_RADIUS);
		b2.setVelocity(new Vector2(-MAX_VELOCITY, 0));
		b2.setMass(10);

		Circle b3 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH / 2 - 40, Globals.MAX_GAME_WIDTH / 2), Globals.DEFAULT_CIRCLE_RADIUS);
		b3.setVelocity(new Vector2(0, 0));
		b3.setMass(25);

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);
		world.addBodyToWorld(b3);
	}

	/* A headon collision (1d) */
	public void makeHeadOn()
	{
		// // Head on collision
		Circle b1 = new Circle(new Vector2(10, Globals.MAX_GAME_WIDTH / 2 - 30), Globals.DEFAULT_CIRCLE_RADIUS);
		b1.setVelocity(new Vector2(MAX_VELOCITY, 0));
		b1.setMass(5);

		Circle b2 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH - 10, Globals.MAX_GAME_WIDTH / 2 - 30), Globals.DEFAULT_CIRCLE_RADIUS);
		b2.setVelocity(new Vector2(-MAX_VELOCITY, 0));
		b2.setMass(10);

		Circle b3 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH / 2 - 40, Globals.MAX_GAME_WIDTH / 2 - 30), Globals.DEFAULT_CIRCLE_RADIUS);
		b3.setVelocity(new Vector2(0, 0));
		b3.setMass(50);

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);
		world.addBodyToWorld(b3);

	}

	public void makeNewtonCradle()
	{
		int centerX = Globals.MAX_GAME_WIDTH / 2;
		int xLoc = (int) ((centerX - BOX_LENGTH * Globals.DEFAULT_CIRCLE_RADIUS) + (Globals.DEFAULT_CIRCLE_RADIUS));
		int yLoc1 = Globals.MAX_GAME_WIDTH / 2;
		Vector2 loc1 = new Vector2(xLoc, yLoc1);
		Vector2 loc2 = new Vector2 (50, yLoc1);

		Circle b1 = new Circle(loc1, Globals.DEFAULT_CIRCLE_RADIUS);
		Circle b2 = new Circle(loc2, Globals.DEFAULT_CIRCLE_RADIUS);
		Circle b3 = new Circle(40, yLoc1, Globals.DEFAULT_CIRCLE_RADIUS);
		Circle b4 = new Circle(centerX - 40, yLoc1, Globals.DEFAULT_CIRCLE_RADIUS);
		b1.setMass(MAX_MASS);
		b2.setMass(MAX_MASS);
		b3.setMass(Globals.INFINITY);
		b4.setMass(Globals.INFINITY);

		b2.setVelocity(new Vector2(100, 0));

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);
		world.addBodyToWorld(b3);
		world.addBodyToWorld(b4);
	}

	/* Make some random circles to bounce around */
	public void makeRandomBodies()
	{	
		//RANDOM BODIES
		for (int i = 0; i < NUM_BODIES; i++)
		{
			Vector2 location = new Vector2(generator.nextInt(Globals.MAX_GAME_WIDTH), generator.nextInt(Globals.MAX_GAME_HEIGHT));
			Vector2 velocity = new Vector2(generator.nextInt(MAX_VELOCITY), generator.nextInt(MAX_VELOCITY));
			int mass = generator.nextInt(MAX_MASS);
			mass = 10;

			Circle b = new Circle(location, Globals.DEFAULT_CIRCLE_RADIUS);

			System.out.println("Velcity:" + velocity.magnitude() + "  X:" + location.x() + " Y:" + location.y());
			b.setVelocity(velocity);
			b.setMass(mass);

			// add the body to physics engine
			world.addBodyToWorld(b);
		} 

		// A single body w/ random velocity in middle
		Vector2 location = new Vector2(Globals.MAX_GAME_WIDTH / 2, Globals.MAX_GAME_HEIGHT / 2);
		Vector2 velocity = new Vector2(generator.nextInt(MAX_VELOCITY), generator.nextInt(MAX_VELOCITY));
		int mass = 10;

		Circle b = new Circle(location, Globals.DEFAULT_CIRCLE_RADIUS);

		b.setVelocity(velocity);
		b.setMass(mass);

		// add the body to physics engine
		world.addBodyToWorld(b);
	}

}



/* JPanel that does all the drawing to the mainFrame */
class DrawingPanel extends JPanel 
{
	private long nextSecond = System.currentTimeMillis() + 1000;
	private int framesInLastSecond = 0;
	private int framesInCurrentSecond = 0;

	public void paintComponent(Graphics g) 
	{

		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D) g;
//		AffineTransform at = g2.getTransform();
		
		// we want to the coordinate system to be standard, have 0,0 in bottom left corner
		//			^  y
		//			|
		//			|______> x
//		g2.translate(0, Globals.MAX_GAME_HEIGHT);
//		g2.scale(1, -1);
		
//		drawOrigin(g2);

		//paint all bodies associated with it
		ArrayList<RigidBody> bodies = World.allBodies();
		for (int j = 0; j < bodies.size(); j++) 
		{
			if (bodies.get(j) == null)
				continue;
			
			if (bodies.get(j).type() == BodyType.CIRCLE)
			{
				Circle c = (Circle) bodies.get(j);
				Rectangle2D.Float bounds = new Rectangle2D.Float(c.center().x() - c.bounds().halfWidth(),
						c.center().y() - c.bounds().halfHeight(), c.bounds().halfWidth() * 2, c.bounds().halfHeight() * 2);
				Ellipse2D.Float geom = new Ellipse2D.Float(c.center().x() - c.bounds().radius(), 
						c.center().y() - c.bounds().radius(), c.bounds().radius() * 2, c.bounds().radius() * 2);
				int processId = c.process();

				switch(processId)
				{
				case 0:
					break;
				case 1:
					g2.setColor(new Color(25, 10, 100));
					break;
				case 2:
					g2.setColor(new Color(100, 10, 25));
					break;
				default:
					g2.setColor(new Color(10, 100, 25));
					break;
				}

				g2.fill(geom);
				g2.draw(bounds);
			}
			else if (bodies.get(j).type() == BodyType.POLYGON)
			{	
				PolyBody b = (PolyBody) bodies.get(j);
				Rectangle2D.Float bounds = new Rectangle2D.Float(b.center().x() - b.bounds().halfWidth(),
						b.center().y() - b.bounds().halfHeight(), b.bounds().halfWidth() * 2, b.bounds().halfHeight() * 2);
//				g2.rotate(b.orientation());
				
				g2.fillPolygon(b.polygonWorld());
				g2.draw(bounds);
				
//				g2.rotate(-b.orientation());
				
				drawNormals(b, g2);
			}
			g2.setColor(new Color(0, 0, 0));
		}

		if (Globals.DRAW_QUADTREE)
			drawQuadTree(World.getTreeRoot(), g2);
		
		for (int i = 0; i < Globals.NUM_PROCESSORS; i++)
		{
			drawContacts(World.getThread(i).contacts, g2);
		}


		// Below (framesInLastSecond) shows the FPS of the main thread
		//		long currentTime = System.currentTimeMillis();
		//		if (currentTime > nextSecond) 
		//		{
		//			nextSecond += 1000;
		//			framesInLastSecond = framesInCurrentSecond;
		//			framesInCurrentSecond = 0;
		//		}
		//		framesInCurrentSecond++;
		//
		//		g2.drawString(framesInLastSecond + " FPS: ", 75, 75);

		// Show fps of each thread, remake back to java coordinates so this displays correctly
//		g2.setTransform(at);
//		for (int i = 0; i < Globals.NUM_PROCESSORS; i++)
//		{
//			g2.drawString("Elapsed time for 1 loop: " + Math.round(World.getThread(i).timeStep * 100), 75, 75 + (i * 15));
//		}
		g2.drawString("NumBodies: " + bodies.size(), 75, 55);
		
	}
	
	public void drawOrigin(Graphics2D g2)
	{
		Line2D.Float xLine = new Line2D.Float(5, 5, 5, 50);
		Line2D.Float yLine = new Line2D.Float(5, 5, 50, 5);
		g2.draw(xLine);
		g2.draw(yLine);
	}

	public void drawContacts(ArrayList<Contact> contacts, Graphics2D g2)
	{
		g2.setColor(new Color(0, 130, 200));
		Vector2 center;
		float radius;
		for (int i = 0; i < contacts.size(); i++)
		{
			center = contacts.get(i).contactPoint();
			radius = Globals.DEFAULT_CIRCLE_RADIUS;
			Ellipse2D.Float geom = new Ellipse2D.Float(center.x() - radius, 
					center.y() - radius, radius * 2, radius * 2);
			g2.draw(geom);
		}
		g2.setColor(new Color(0, 0, 0));
	}
	
	public void drawQuadTree(QuadTreeNode n, Graphics2D g2)
	{
		if (n == null)
			return;

		// Draw the parent node
		g2.drawRect(n.bounds().x, n.bounds().y, n.bounds().width, n.bounds().height);

//		if (n.bodies().size() > 0)
//			System.out.println(" Depth: " + n.depth() + "   numbodies: " + n.bodies().size());
		
		// draw all children of parent node
		for (int i = 0; i < 4; i++)
		{
			if (n.children()[i] != null)
				drawQuadTree(n.children()[i], g2);
		}
	}
	
	// draw the normals onto the body
	public void drawNormals(PolyBody b, Graphics2D g2)
	{
		g2.setColor(new Color(130, 200, 0));
		for (int i = 0; i < b.numVertices(); i++)
		{
			Vector2 normal = b.normal(i); 
			float fromx = b.center().x();
			float fromy = b.center().y();
			float tox = normal.x() + fromx;
			float toy = normal.y() + fromy;
			
//			System.out.println(" V1: " + b.verticesWorld().get(i) + "   v2: " + b.verticesWorld().get( (i + 1) % b.numVertices()) + "   normal: " + normal);
			Line2D.Float line = new Line2D.Float(fromx, fromy, tox, toy);
			g2.draw(line);
		}
		g2.setColor(new Color(0, 0, 0));
	}

}


/* Timer class to take one step every x seconds */
class PhysicsStep extends TimerTask 
{
	private World world;
	private DrawingPanel drawingPanel;

	public PhysicsStep(World w, JPanel dp)
	{
		drawingPanel = (DrawingPanel)dp;
		world = w;
	}

	@Override
	public void run() 
	{
		//System.out.println("Step");
		drawingPanel.repaint();

	}
}


class ClickListener implements MouseListener
{

	@Override
	public void mouseClicked(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseEntered(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseExited(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mousePressed(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseReleased(MouseEvent arg0) 
	{
		if (arg0.getButton() == MouseEvent.BUTTON1)
		{
			// Add an object to the location
			int centerX = arg0.getX();
			int centerY = arg0.getY();

			int[] xpoints = {-10, 10, 10, -10};
			int[] ypoints = {-10, -10, 10, 10};

			Polygon p = new Polygon(xpoints, ypoints, 4);
			Vector2 loc = new Vector2(centerX, centerY);

			//Circle b1 = new Circle(loc, Globals.DEFAULT_CIRCLE_RADIUS);
			PolyBody b1 = new PolyBody(p, loc);
			b1.setMass(10);

			Vector2 a = new Vector2(0, Globals.DEFAULT_GRAVITY);
			b1.setAcceleration(a);

			System.out.println("Click added @ : <" + centerX + ", " + centerY);

			ExampleGame.world.addBodyToWorld(b1);
		}
		else
		{
			// Add an object to the location
			int centerX = arg0.getX();
			int centerY = arg0.getY();

			int[] xpoints = {-10, 0, 10, 10, 0, -10};
			int[] ypoints = {-10, -20, -10, 10, 20, 10};

			Polygon p = new Polygon(xpoints, ypoints, 6);
			Vector2 loc = new Vector2(centerX, centerY);
			
//			int[] xpoints = {-10, 10, 10, -10};
//			int[] ypoints = {-10, -10, 10, 10};

//			Polygon p = new Polygon(xpoints, ypoints, 4);
//			Vector2 loc = new Vector2(centerX, centerY);

			//Circle b1 = new Circle(loc, Globals.DEFAULT_CIRCLE_RADIUS);
			PolyBody b1 = new PolyBody(p, loc);
			b1.setMass(10);

			Vector2 a = new Vector2(0, Globals.DEFAULT_GRAVITY);
			Vector2 v = new Vector2(100, 0);
			b1.setVelocity(v);
//			b1.setAcceleration(a);

			System.out.println("Click added @ : <" + centerX + ", " + centerY);

			ExampleGame.world.addBodyToWorld(b1);
		}
	}

}
