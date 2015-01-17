package com.sjsu.physics.core;

import java.util.ArrayList;

import com.sjsu.physics.utils.Globals;

public class ContactSolver
{
	private int maxIterations;
	private int curIteration;

	public ContactSolver(int iterations)
	{
		maxIterations = iterations;
		curIteration = 0;
	}

	/** Check to see if the objects have moved apart enough to remove the contact */
	public boolean canRemoveContact(Contact c)
	{
		if (c.velocityAlongNormal() > 0)
			return false; // objects are moving towards each other

		if (c.a().center().distanceTo(c.b().center()) < Globals.DISTANCE_TO_REMOVE)
			return true;

		return false;
	}

	/**
	 *  Resolve a list of contacts
	 * TODO we should be resolving contacts based on highest velocity first to reduce adding noise
	 */
	public void resolveContacts(ArrayList<Contact> contacts, float time)
	{
		curIteration = 0;

		for (int i = 0; i < contacts.size(); i++)
		{
			contacts.get(i).resolve(time);
			contacts.remove(i);
		}
	}

}
