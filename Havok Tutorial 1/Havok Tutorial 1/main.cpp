// Havok works in metres and kilograms for its units. If you want to work with different measurements,
// you have to scale everything appropriately. So if a cube is now defined as 1x1x1 feet instead of metres
// you would need to change gravity to work with feet instead of metres, and so forth. Havok was designed to
// be most accurate with numbers close to a magnitude of 1 (so 1000000 and 0.000001 are bad). Try to stick to
// metres if you can. (from http://graphics.ethz.ch/Downloads/Seminar_Arbeiten/2002_03/Havok_Overview.pdf)

#include "HavokCore.h"

#include <Physics2012/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics2012/Dynamics/Entity/hkpRigidBody.h>
#include <Physics2012/Utilities/Dynamics/Inertia/hkpInertiaTensorComputer.h>

void addGroundBox(hkpWorld* world);
void addFallingBox(hkpWorld* world);

int HK_CALL main(int argc, const char** argv)
{
	HavokCore* havok = new HavokCore();

	// Add a box to act as the ground and a box slightly above it to fall down onto the ground box thanks to gravity.
	addGroundBox(havok->getWorld());
	addFallingBox(havok->getWorld());

	// A stopwatch for waiting until real time has passed.
	hkStopwatch stopWatch;
	stopWatch.start();

	hkReal lastTime = stopWatch.getElapsedSeconds();

	// Update as if running at 60 FPS.
	hkReal timeStep = 1.0f / 60.0f;
	// Run for 20 seconds.
	const int numSteps = int(20.0f / timeStep);

	// Run the simulation for the number of steps calculated.
	for (int i = 0; i < numSteps; ++i)
	{
		havok->stepSimulation(timeStep);

		// Pause until the actual time has passed.
		while (stopWatch.getElapsedSeconds() < lastTime + timeStep) { }
		lastTime += timeStep;
	}

	// Clean everything up.
	delete havok;

	return 0;
}

void addGroundBox(hkpWorld* world)
{
	// Create a ground area.
	hkVector4 halfExtents(20.0f, 1.0f, 20.0f);
	hkpBoxShape* boxShape = new hkpBoxShape(halfExtents);

	// Set its properties.
	hkpRigidBodyCinfo ci;
	ci.m_shape = boxShape; // set its rigid body as shaped like a box.
	ci.m_position = hkVector4(0.0f, 0.0f, 0.0f); // set its position at the origin
	ci.m_motionType = hkpMotion::MOTION_FIXED; // set its motion as fixed, meaning it will never move

	// Create the rigid body.
	hkpRigidBody* rigidBody = new hkpRigidBody(ci);

	// No longer need the reference on the boxShape, as the rigidBody now owns it.
	boxShape->removeReference();

	// Remove  rigidBody's reference and add it to the world.
	world->addEntity(rigidBody)->removeReference();
}

void addFallingBox(hkpWorld* world)
{
	// Create a box 1 by 2 by 3.
	hkVector4 halfExtents(0.5f, 1.0f, 1.5f);
	hkpBoxShape* boxShape = new hkpBoxShape(halfExtents);

	// Set the box's properties.
	hkpRigidBodyCinfo ci;
	ci.m_shape = boxShape; // set its rigid body as shaped like a box.
	ci.m_position = hkVector4(0.0f, 50.0f, 0.0f); // set its position slightly above the ground box
	ci.m_motionType = hkpMotion::MOTION_DYNAMIC; // set its motion as dynamic, meaning that it is able to move

	// Calculate the mass properties for the shape and add them to the box's properties.
	const hkReal boxMass = 10.0f;
	hkMassProperties massProperties;
	hkInertiaTensorComputer::computeBoxVolumeMassProperties(halfExtents, boxMass, massProperties);

	ci.setMassProperties(massProperties);

	// Create the rigid body.
	hkpRigidBody* rigidBody = new hkpRigidBody(ci);

	// We no longer need the reference on boxShape as the rigidBody now owns it.
	boxShape->removeReference();

	// Remove  rigidBody's reference and add it to the world.
	world->addEntity(rigidBody)->removeReference();
}