package com.sjsu.physics.forcegenerators;

import com.sjsu.physics.shapes.RigidBody;

/*
 * Interface for a force. Different forces
 * can implement this interface as needed.
 * Useful for forces applied to rigid-bodies
 * for long periods of time that may need to be changed
 * (ie acceleration fields, drag forces, etc).
 */
public interface Force
{
	void updateForce(RigidBody body, float t);
}
