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

	private static final int BOX_LENGTH = 8;
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
		makeFixedBox();
		makeRandomBodies();
		//makeHeadOn();
		//makeOffsetHeadOn();
		//makePolygons();
		//makeHeadOnPolygons();
		//makeFloor();
		//makeRotation();
		//makeNewtonCradle();
		//makeRestitution();
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


	/* create a box of bodies to act as a wall-bounding box fixed */
	public void makeFixedBox()
	{
		int centerX = Globals.MAX_GAME_WIDTH / 2;
		int centerY = Globals.MAX_GAME_WIDTH / 2;
		int width = (Globals.MAX_GAME_WIDTH / 16);
		int height = (Globals.MAX_GAME_HEIGHT / 16);
		int[] xpoints1 = {-width - 1, width, width, -width - 1};
		int[] ypoints1 = {-10, -10, 10, 10};
		int[] xpoints2 = {-10, 10, 10, -10};
		int[] ypoints2 = {-height - 1, -height - 1, height, height};
		
		Polygon pV = new Polygon(xpoints2, ypoints2, 4);
		Polygon pH = new Polygon(xpoints1, ypoints1, 4);
		
		// Verticle walls
		for (int i = 0; i < BOX_LENGTH; i++)
		{	
			int xLoc = (int) (i * ( (height) * 2) + height + (i)); 
			int yLoc1 = (int) (5);
			int yLoc2 = (int) (Globals.MAX_GAME_WIDTH - 5);

			PolyBody b1 = new PolyBody(pH, new Vector2(xLoc, yLoc1));
			PolyBody b2 = new PolyBody(pH, new Vector2(xLoc, yLoc2));
			
			b1.setMass(Globals.INFINITY);
			b2.setMass(Globals.INFINITY);

			world.addBodyToWorld(b1);
			world.addBodyToWorld(b2);
		}

		// Horizontal walls
		for (int i = 0; i < BOX_LENGTH; i++)
		{
			int yLoc = (int)  (i * ( (width) * 2) + width + i); 
			int xLoc1 = (int) (5);
			int xLoc2 = (int) (Globals.MAX_GAME_HEIGHT - 5);

			
			PolyBody b1 = new PolyBody(pV, new Vector2(xLoc1, yLoc));
			PolyBody b2 = new PolyBody(pV, new Vector2(xLoc2, yLoc));
			
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
		PolyBody b2 = new PolyBody(p2, 400, 630);

		b1.setVelocity(Globals.ZERO_VECTOR);
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
		int[] xpoints4 = {-10, 10, 10, -10};
		int[] ypoints4 = {-200, -200, 200, 200};

		Polygon p1 = new Polygon(xpoints1, ypoints1, 4);
		Polygon p2 = new Polygon(xpoints2, ypoints2, 4);
		Polygon p3 = new Polygon(xpoints3, ypoints3, 4);
		Polygon p4 = new Polygon(xpoints4, ypoints4, 4);

		PolyBody b1 = new PolyBody(p1, 300, Globals.MAX_GAME_HEIGHT - 400);
		PolyBody b2 = new PolyBody(p2, 600, Globals.MAX_GAME_HEIGHT - 400);
		PolyBody b3 = new PolyBody(p3, 550, Globals.MAX_GAME_HEIGHT - 700);
		PolyBody b4 = new PolyBody(p4, 730, Globals.MAX_GAME_HEIGHT - 200);

		b1.setMass(Globals.INFINITY);
		b2.setMass(Globals.INFINITY);
		b3.setMass(Globals.INFINITY);
		b4.setMass(Globals.INFINITY);
		
		b4.rotateBy(1);

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);
		world.addBodyToWorld(b3);
		world.addBodyToWorld(b4);
	}
	
	/* a floor polygon */
	public void makeRotation()
	{
		int[] xpoints1 = {-100, 100, 100, -100};
		int[] ypoints1 = {-10, -10, 10, 10};
		int[] xpoints3 = {-10, 10, 10, -10};
		int[] ypoints3 = {-100, -100, 100, 100};

		Polygon p1 = new Polygon(xpoints1, ypoints1, 4);
		Polygon p3 = new Polygon(xpoints3, ypoints3, 4);

		PolyBody b1 = new PolyBody(p1, 300 - 60, Globals.MAX_GAME_HEIGHT - 600);
		PolyBody b2 = new PolyBody(p1, 700, Globals.MAX_GAME_HEIGHT - 600);
		PolyBody b3 = new PolyBody(p3, 500 - 30, Globals.MAX_GAME_HEIGHT - 800);

		b1.setMass(Globals.INFINITY);
		b3.setMass(Globals.INFINITY);
		b2.setMass(Globals.INFINITY);
		
		world.addBodyToWorld(b2);
		world.addBodyToWorld(b1);
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
		b3.setVelocity(Globals.ZERO_VECTOR);
		b3.setMass(25);

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);
		world.addBodyToWorld(b3);
	}

	/* A headon collision (1d) */
	public void makeHeadOn()
	{
		// // Head on collision
		Circle b1 = new Circle(new Vector2(20, Globals.MAX_GAME_WIDTH / 2 - 300), 4);
		b1.setVelocity(new Vector2(MAX_VELOCITY - 5, 0));
		b1.setMass(3);

		Circle b2 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH - 20, Globals.MAX_GAME_WIDTH / 2 - 300), 8);
		b2.setVelocity(new Vector2(-MAX_VELOCITY + 5, 0));
		b2.setMass(12);

		Circle b3 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH / 2 - 100, Globals.MAX_GAME_WIDTH / 2 - 300), 11);
		b3.setVelocity(new Vector2(0, 0));
		b3.setMass(47);
		
		// // Head on collision
		Circle a1 = new Circle(new Vector2(20, Globals.MAX_GAME_WIDTH / 2 + 300), 6);
		a1.setVelocity(new Vector2(MAX_VELOCITY, 0));
		a1.setMass(7);

		Circle a2 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH - 20, Globals.MAX_GAME_WIDTH / 2 + 300), 7);
		a2.setVelocity(new Vector2(-MAX_VELOCITY, 0));
		a2.setMass(12);

		Circle a3 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH / 2 - 90, Globals.MAX_GAME_WIDTH / 2 + 300), 11);
		a3.setVelocity(new Vector2(0, 0));
		a3.setMass(68);
		
		// // Head on collision
		Circle c1 = new Circle(new Vector2(20, Globals.MAX_GAME_WIDTH / 2 + 100), 5);
		c1.setVelocity(new Vector2(MAX_VELOCITY, 0));
		c1.setMass(8);

		Circle c2 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH - 20, Globals.MAX_GAME_WIDTH / 2 + 100), 9);
		c2.setVelocity(new Vector2(-MAX_VELOCITY, 0));
		c2.setMass(30);

		Circle c3 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH / 2 - 110, Globals.MAX_GAME_WIDTH / 2 + 100), 6);
		c3.setVelocity(new Vector2(0, 0));
		c3.setMass(13);

		// // Head on collision
		Circle d1 = new Circle(new Vector2(20, Globals.MAX_GAME_WIDTH / 2 - 100), 8);
		d1.setVelocity(new Vector2(MAX_VELOCITY, 0));
		d1.setMass(10);

		Circle d2 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH - 20, Globals.MAX_GAME_WIDTH / 2 - 100), 8);
		d2.setVelocity(new Vector2(-MAX_VELOCITY, 0));
		d2.setMass(10);

		Circle d3 = new Circle(new Vector2(Globals.MAX_GAME_WIDTH / 2 - 120, Globals.MAX_GAME_WIDTH / 2 - 100), 8);
		d3.setVelocity(new Vector2(0, 0));
		d3.setMass(10);

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);
		world.addBodyToWorld(b3);
		world.addBodyToWorld(a1);
		world.addBodyToWorld(a2);
		world.addBodyToWorld(a3);
		world.addBodyToWorld(c1);
		world.addBodyToWorld(c2);
		world.addBodyToWorld(c3);
		world.addBodyToWorld(d1);
		world.addBodyToWorld(d2);
		world.addBodyToWorld(d3);
		
		
		int[] xpoints1 = {-3, 3, 3, -3};
		int[] ypoints1 = {-200, -200, 200, 200};

		Polygon p1 = new Polygon(xpoints1, ypoints1, 4);

		PolyBody h1 = new PolyBody(p1, 10, Globals.MAX_GAME_HEIGHT / 2 + 250);
		PolyBody h2 = new PolyBody(p1, Globals.MAX_GAME_WIDTH - 10, Globals.MAX_GAME_HEIGHT / 2 - 250);
		PolyBody h4 = new PolyBody(p1, Globals.MAX_GAME_WIDTH - 10, Globals.MAX_GAME_HEIGHT / 2 + 250);
		PolyBody h3 = new PolyBody(p1, 10, Globals.MAX_GAME_HEIGHT / 2 - 250);

		h1.setMass(Globals.INFINITY);
		h2.setMass(Globals.INFINITY);
		h4.setMass(Globals.INFINITY);
		h3.setMass(Globals.INFINITY);
		
		world.addBodyToWorld(h1);
		world.addBodyToWorld(h2);
		world.addBodyToWorld(h3);
		world.addBodyToWorld(h4);
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
	
	public void makeRestitution()
	{
		int[] xpoints1 = {-200, 200, 200, -200};
		int[] ypoints1 = {-10, -10, 10, 10};
		int[] xpoints2 = {-200, 200, 200, -200};
		int[] ypoints2 = {-10, -10, 10, 10};
		
		Polygon p1 = new Polygon(xpoints1, ypoints1, 4);
		Polygon p2 = new Polygon(xpoints2, ypoints2, 4);

		PolyBody b1 = new PolyBody(p1, 700, Globals.MAX_GAME_HEIGHT - 100);
		PolyBody b2 = new PolyBody(p1, 300, Globals.MAX_GAME_HEIGHT - 100);

		b1.setMass(Globals.INFINITY);
		b2.setMass(Globals.INFINITY);

		world.addBodyToWorld(b1);
		world.addBodyToWorld(b2);

		int[] xpoints = {-10, 10, 10, -10};
		int[] ypoints = {-10, -10, 10, 10};

		Polygon s1 = new Polygon(xpoints, ypoints, 4);

		PolyBody sq1 = new PolyBody(s1, new Vector2(125, 100));
		sq1.setMass(10);
		PolyBody sq2 = new PolyBody(s1, new Vector2(175, 105));
		sq2.setMass(10);
		PolyBody sq3 = new PolyBody(s1, new Vector2(225, 110));
		sq3.setMass(10);
		PolyBody sq4 = new PolyBody(s1, new Vector2(275, 115));
		sq4.setMass(10);
		PolyBody sq5 = new PolyBody(s1, new Vector2(325, 120));
		sq5.setMass(10);

		Vector2 a = new Vector2(0, Globals.DEFAULT_GRAVITY / 2);
		sq1.setAcceleration(a);
		sq2.setAcceleration(a);
		sq3.setAcceleration(a);
		sq4.setAcceleration(a);
		sq5.setAcceleration(a);
		

		ExampleGame.world.addBodyToWorld(sq1);
		ExampleGame.world.addBodyToWorld(sq2);
		ExampleGame.world.addBodyToWorld(sq3);
		ExampleGame.world.addBodyToWorld(sq4);
		ExampleGame.world.addBodyToWorld(sq5);
		

		Circle c1 = new Circle(new Vector2(625, 100), 10);
		c1.setMass(5);
		c1.setAcceleration(a);
		Circle c2 = new Circle(new Vector2(675, 105), 10);
		c2.setMass(5);
		c2.setAcceleration(a);
		Circle c3 = new Circle(new Vector2(725, 110), 10);
		c3.setMass(5);
		c3.setAcceleration(a);
		Circle c4 = new Circle(new Vector2(775, 115), 10);
		c4.setMass(5);
		c4.setAcceleration(a);
		Circle c5 = new Circle(new Vector2(825, 120), 10);
		c5.setMass(5);
		c5.setAcceleration(a);

		ExampleGame.world.addBodyToWorld(c1);
		ExampleGame.world.addBodyToWorld(c2);
		ExampleGame.world.addBodyToWorld(c3);
		ExampleGame.world.addBodyToWorld(c4);
		ExampleGame.world.addBodyToWorld(c5);
		
//		Circle c1 = new Circle(new Vector2(110, Globals.MAX_GAME_WIDTH - 650), 5 * Globals.DEFAULT_CIRCLE_RADIUS);
//		c1.setVelocity(new Vector2(35, 0));
//		c1.setAcceleration(new Vector2(0, Globals.DEFAULT_GRAVITY));
//		c1.setMass(5);
//		world.addBodyToWorld(c1);
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
	private static Color COLORBLACK = new Color(0, 0, 0);
	private static Color COLORP0 = new Color(132, 161, 240);
	private static Color COLORP1 = new Color(21, 92, 73);
	private static Color COLORP2 = new Color(21, 40, 92);
	private static Color COLORP3 = new Color(145, 51, 57);
	
	private long nextSecond = System.currentTimeMillis() + 1000;
	private int framesInLastSecond = 0;
	private int framesInCurrentSecond = 0;

	public void paintComponent(Graphics g) 
	{

		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D) g;

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
				
				g2.setColor(getColor((RigidBody) c));
				g2.fill(geom);
				g2.draw(bounds);
			}
			else if (bodies.get(j).type() == BodyType.POLYGON)
			{	
				PolyBody b = (PolyBody) bodies.get(j);
				Rectangle2D.Float bounds = new Rectangle2D.Float(b.center().x() - b.bounds().halfWidth(),
						b.center().y() - b.bounds().halfHeight(), b.bounds().halfWidth() * 2, b.bounds().halfHeight() * 2);

				g2.setColor(getColor((RigidBody) b));
				g2.fillPolygon(b.polygonWorld());
				g2.setColor(COLORBLACK);
				g2.draw(bounds);
								
				//drawNormals(b, g2);
			}
			g2.setColor(COLORBLACK);
		}

		if (Globals.DRAW_QUADTREE)
			drawQuadTree(World.getTreeRoot(), g2);
		
//		for (int i = 0; i < Globals.NUM_PROCESSORS; i++)
//		{
//			drawContacts(World.getThread(i).contacts, g2);
//		}


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
	
	// Get the color based on process number
	private Color getColor(RigidBody a)
	{
		int processId = a.process();

		switch(processId)
		{
		case 0:
			return COLORP0;
		case 1:
			return COLORP1;
		case 2:
			return COLORP2;
		default:
			return COLORP3;
		}
	}
	
	private void drawOrigin(Graphics2D g2)
	{
		Line2D.Float xLine = new Line2D.Float(5, 5, 5, 50);
		Line2D.Float yLine = new Line2D.Float(5, 5, 50, 5);
		g2.draw(xLine);
		g2.draw(yLine);
	}

	private void drawContacts(ArrayList<Contact> contacts, Graphics2D g2)
	{
		g2.setColor(new Color(0, 130, 200));
		Vector2 center;
		Vector2 center1;
		float radius;
		for (int i = 0; i < contacts.size(); i++)
		{
			Vector2 normal = contacts.get(i).normal().multiplyBy(10);
			float fromx = contacts.get(i).contactPoint().x();
			float fromy = contacts.get(i).contactPoint().y();
			float tox = normal.x() + fromx;
			float toy = normal.y() + fromy;
			
//			System.out.println(" V1: " + b.verticesWorld().get(i) + "   v2: " + b.verticesWorld().get( (i + 1) % b.numVertices()) + "   normal: " + normal);
			Line2D.Float line = new Line2D.Float(fromx, fromy, tox, toy);
			g2.draw(line);
			
			center = contacts.get(i).contactPoint();
			radius = Globals.DEFAULT_CIRCLE_RADIUS;
			Ellipse2D.Float geom = new Ellipse2D.Float(center.x() - radius, 
					center.y() - radius, radius * 2, radius * 2);
			g2.draw(geom);
		}
		g2.setColor(new Color(0, 0, 0));
	}
	
	private void drawQuadTree(QuadTreeNode n, Graphics2D g2)
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
			float radius = 5;
			Ellipse2D.Float center = new Ellipse2D.Float(b.center().x() - radius, 
					b.center().y() - radius, radius * 2, radius * 2);
			g2.draw(line);
			g2.draw(center);
		}
		g2.setColor(COLORBLACK);
	}

}


/* Timer class to take one step every x seconds */
class PhysicsStep extends TimerTask 
{
	private World world;
	private DrawingPanel drawingPanel;
	private static long nextSecond = System.currentTimeMillis() + 1000;
	private Random rand;

	public PhysicsStep(World w, JPanel dp)
	{
		drawingPanel = (DrawingPanel)dp;
		world = w;
		rand = new Random();
	}

	@Override
	public void run() 
	{
		//System.out.println("Step");
		drawingPanel.repaint();
//		makeBody();
	}
	
	public void makeBody()
	{
		long currentTime = System.currentTimeMillis();
		if (currentTime > nextSecond) 
		{
			nextSecond += 5000;
			
			int smallRand = rand.nextInt(10);
			float restitution = rand.nextFloat();
			smallRand -= 5;

			int[] xpoints = {-10, 10, 10, -10};
			int[] ypoints = {-10, -10, 10, 10};

			Polygon p = new Polygon(xpoints, ypoints, 4);
			Vector2 loc = new Vector2(500 - 30 + (14 * Math.signum(smallRand)), 30);

			PolyBody b1 = new PolyBody(p, loc);
			b1.setMass(20 * restitution);

			Vector2 a = new Vector2(0, Globals.DEFAULT_GRAVITY);
			b1.setAcceleration(a);

			world.addBodyToWorld(b1);
		}
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
		if (arg0.isShiftDown() && arg0.getButton() == MouseEvent.BUTTON1)
		{
			int centerX = arg0.getX();
			int centerY = arg0.getY();
			
			int[] xpoints = {-50, 50, 50, -50};
			int[] ypoints = {-10, -10, 10, 10};

			Polygon p = new Polygon(xpoints, ypoints, 4);
			Vector2 loc = new Vector2(centerX, centerY);

			PolyBody b1 = new PolyBody(p, loc);
			b1.setMass(100);
			b1.rotateBy(1);

			Vector2 a = new Vector2(0, Globals.DEFAULT_GRAVITY);
			b1.setAcceleration(a);

			System.out.println("Click added @ : <" + centerX + ", " + centerY);

			ExampleGame.world.addBodyToWorld(b1);
		}
		else if (arg0.getButton() == MouseEvent.BUTTON1)
		{
			int centerX = arg0.getX();
			int centerY = arg0.getY();

			int[] xpoints = {-10, 10, 10, -10};
			int[] ypoints = {-10, -10, 10, 10};

			Polygon p = new Polygon(xpoints, ypoints, 4);
			Vector2 loc = new Vector2(centerX, centerY);

			PolyBody b1 = new PolyBody(p, loc);
			b1.setMass(10);

			Vector2 a = new Vector2(0, Globals.DEFAULT_GRAVITY);
			b1.setAcceleration(a);

			System.out.println("Click added @ : <" + centerX + ", " + centerY);

			ExampleGame.world.addBodyToWorld(b1);
		}
		else if (arg0.getButton() == MouseEvent.BUTTON3 && arg0.isShiftDown())
		{
			int centerX = arg0.getX();
			int centerY = arg0.getY();

			int[] xpoints = {-10, 0, 10, 10, 0, -10};
			int[] ypoints = {-10, -20, -10, 10, 20, 10};

			Polygon p = new Polygon(xpoints, ypoints, 6);
			Vector2 loc = new Vector2(centerX, centerY);

			PolyBody b1 = new PolyBody(p, loc);
			b1.setMass(20);

			Vector2 a = new Vector2(0, Globals.DEFAULT_GRAVITY);
			Vector2 v = new Vector2(10, 0);
			b1.setVelocity(v);
			b1.setAcceleration(a);

			System.out.println("Click added @ : <" + centerX + ", " + centerY);

			ExampleGame.world.addBodyToWorld(b1);
		}
		else if (arg0.getButton() == MouseEvent.BUTTON3)
		{
			int centerX = arg0.getX();
			int centerY = arg0.getY();

			int[] xpoints = {-10, 10, 10, -10};
			int[] ypoints = {-10, -10, 10, 10};

			Polygon p = new Polygon(xpoints, ypoints, 4);
			Vector2 loc = new Vector2(centerX, centerY);

			Circle b1 = new Circle(loc, 10);
			b1.setMass(5);

			Vector2 a = new Vector2(0, Globals.DEFAULT_GRAVITY);
			b1.setAcceleration(a);
			b1.setVelocity(40, 0);

			System.out.println("Click added @ : <" + centerX + ", " + centerY);

			ExampleGame.world.addBodyToWorld(b1);
		}
	}

}
