package com.sjsu.physics.core;

import java.util.ArrayList;

import com.sjsu.physics.utils.Globals;

public class ContactSolver
{
	private int maxIterations; // Max number of interations allowed when
								// resolving
	private int curIteration; // Cur number of iterations we have used

	public ContactSolver(int iterations)
	{
		maxIterations = iterations;
		curIteration = 0;
	}

	public void setMaxIterations(int i)
	{
		maxIterations = i;
	}

	/* Check to see if the objects have moved apart enough to remove the contact */
	public boolean canRemoveContact(Contact c)
	{
		if (c.velocityAlongNormal() > 0)
			return false; // objects are moving towards each other

		if (c.a().center().distanceTo(c.b().center()) < Globals.DISTANCE_TO_REMOVE)
			return true;

		return false;
	}

	/* Resolve a list of contacts */
	public void resolveContacts(ArrayList<Contact> contacts, float time)
	{
		// System.out.println("Num contacts: " + contacts.size());
		curIteration = 0;

		for (int i = 0; i < contacts.size(); i++)
		{
			contacts.get(i).resolve(time);
			contacts.remove(i);
		}

		// /* Resolve contacts on priority of largest closing velocity to
		// smallest closing velocity */
		// while (curIteration < maxIterations)
		// {
		// float maxV = Globals.INFINITY;
		// int maxIndex = contacts.size() - 1;
		// for (int i = 0; i < contacts.size(); i++)
		// {
		// float sepVelocity = contacts.get(i).velocityAlongNormal();
		// if (sepVelocity < maxV && sepVelocity < 0 &&
		// contacts.get(i).penetration() > 0)
		// {
		// maxV = sepVelocity;
		// maxIndex = i;
		// }
		// }
		//
		// // if we have nothing worth resolving, exit early
		// if (maxIndex == contacts.size() - 1)
		// break;
		//
		// /* Resolve the max found contact */
		// contacts.get(maxIndex).resolve(time);
		//
		// // update penetration for all contacts
		// // Update the interpenetrations for all particles
		// Vector2 moveA =
		// contacts.get(maxIndex).particleMovement(contacts.get(maxIndex).a());
		// Vector2 moveB =
		// contacts.get(maxIndex).particleMovement(contacts.get(maxIndex).b());
		//
		// for (int i = 0; i < contacts.size(); i++)
		// {
		// if (contacts.get(i).a() == contacts.get(maxIndex).a())
		// {
		// float newPen = contacts.get(i).penetration() -
		// moveA.dot(contacts.get(i).normal());
		// contacts.get(i).setPenetration(newPen);
		// }
		// else if (contacts.get(i).a() == contacts.get(maxIndex).b())
		// {
		// float newPen = contacts.get(i).penetration() -
		// moveB.dot(contacts.get(i).normal());
		// contacts.get(i).setPenetration(newPen);
		// }
		// if (contacts.get(i).b() == contacts.get(maxIndex).a())
		// {
		// float newPen = contacts.get(i).penetration() -
		// moveA.dot(contacts.get(i).normal());
		// contacts.get(i).setPenetration(newPen);
		// }
		// else if (contacts.get(i).b() == contacts.get(maxIndex).b())
		// {
		// float newPen = contacts.get(i).penetration() -
		// moveB.dot(contacts.get(i).normal());
		// contacts.get(i).setPenetration(newPen);
		// }
		// }
		//
		// curIteration++;
		// }
	}

}
