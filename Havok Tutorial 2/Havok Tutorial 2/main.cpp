// Havok works in metres and kilograms for its units. If you want to work with different measurements,
// you have to scale everything appropriately. So if a cube is now defined as 1x1x1 feet instead of metres
// you would need to change gravity to work with feet instead of metres, and so forth. Havok was designed to
// be most accurate with numbers close to a magnitude of 1 (so 1000000 and 0.000001 are bad). Try to stick to
// metres if you can. (from http://graphics.ethz.ch/Downloads/Seminar_Arbeiten/2002_03/Havok_Overview.pdf)

#include "HavokCore.h"

#include <Physics2012/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics2012/Collide/Shape/Convex/Sphere/hkpSphereShape.h>
#include <Physics2012/Collide/Shape/Convex/Cylinder/hkpCylinderShape.h>
#include <Physics2012/Collide/Shape/Convex/Capsule/hkpCapsuleShape.h>
#include <Physics2012/Collide/Shape/Convex/Triangle/hkpTriangleShape.h>
#include <Physics2012/Collide/Shape/Convex/ConvexVertices/hkpConvexVerticesShape.h>

#include <Physics2012/Dynamics/Entity/hkpRigidBody.h>

#include <Physics2012/Utilities/Dynamics/Inertia/hkpInertiaTensorComputer.h>

#include <Common/Internal/ConvexHull/hkGeometryUtility.h>

void addGroundBox(hkpWorld* world);
void addRigidBody(hkpWorld* world, const hkpRigidBodyCinfo& ci);

void addFallingShapes(hkpWorld* world);

void motionTypes(hkpWorld* world);

int HK_CALL main(int argc, const char** argv)
{
	HavokCore* havok = new HavokCore();

	// Add a box to act as the ground.
	addGroundBox(havok->getWorld());

	//addFallingShapes(havok->getWorld());
	motionTypes(havok->getWorld());

	// A stopwatch for waiting until real time has passed.
	hkStopwatch stopWatch;
	stopWatch.start();

	hkReal lastTime = stopWatch.getElapsedSeconds();

	// Update as if running at 60 FPS.
	hkReal timeStep = 1.0f / 60.0f;
	// Run for 10 seconds.
	const int numSteps = int(10.0f / timeStep);

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


void addRigidBody(hkpWorld* world, const hkpRigidBodyCinfo& ci)
{
	// Create the rigid body.
	hkpRigidBody* rigidBody = new hkpRigidBody(ci);
	rigidBody->getMaterial().setFriction(0.0f);

	// Remove rigidBody's reference and add it to the world.
	world->addEntity(rigidBody)->removeReference();
}

// Create a ground area.
void addGroundBox(hkpWorld* world)
{
	hkVector4 halfExtents(20.0f, 1.0f, 20.0f);
	hkpBoxShape* boxShape = new hkpBoxShape(halfExtents);

	// Set its properties.
	hkpRigidBodyCinfo ci;
	ci.m_shape = boxShape; // set its rigid body as shaped like a box.
	ci.m_position = hkVector4(0.0f, 0.0f, 0.0f); // set its position at the origin. Position is by default at the origin but we'll be explicit.
	ci.m_motionType = hkpMotion::MOTION_FIXED; // set its motion as fixed, meaning it will never move

	// Add the rigid body to the world.
	addRigidBody(world, ci);

	// No longer need the reference on the boxShape, as the rigidBody now owns it.
	boxShape->removeReference();
}

// Add various convex shapes and make them fall on top of one another.
void addFallingShapes(hkpWorld* world)
{
	const hkReal mass = 10.0f;
	const hkVector4 gap = hkVector4(6.0f, 0.0f, 0.0f);
	hkVector4 position = hkVector4(-15.0f, 20.0f, 0.0f);

	// Set universal properties for all the rigid bodies. Motion type does not have to be explicitly defined here
	// as by default it is dynamic but to make things obvious it is done so.
	hkpRigidBodyCinfo ci;
	ci.m_motionType = hkpMotion::MOTION_DYNAMIC;

	// Create and add a box.
	{
		// Define its half-extents. Just 1 metre in x, y, z.
		const hkVector4 halfExtents(1.0f, 1.0f, 1.0f);
		hkpBoxShape* boxShape = new hkpBoxShape(halfExtents);

		// Set its properties.
		ci.m_shape = boxShape;
		ci.m_position = position;

		// Calculate the mass properties for the shape and add them to the rigid body's properties.
		hkMassProperties massProperties;
		hkInertiaTensorComputer::computeBoxVolumeMassProperties(halfExtents, mass, massProperties);
		ci.setMassProperties(massProperties);

		// Add the rigid body to the world.
		addRigidBody(world, ci);
		
		// No longer need the reference on the boxShape, as the rigidBody now owns it.
		boxShape->removeReference();
	}

	position.add(gap);

	// Create and add a sphere.
	{
		// Define a sphere with a 1 metre radius.
		const hkReal radius = 1.0f;
		hkpSphereShape* sphereShape = new hkpSphereShape(radius);

		// Set its properties.
		ci.m_shape = sphereShape;
		ci.m_position = position;

		// Calculate the mass properties for the shape and add them to the rigid body's properties;
		hkMassProperties massProperties;
		hkInertiaTensorComputer::computeSphereVolumeMassProperties(radius, mass, massProperties);
		ci.setMassProperties(massProperties);

		// Add the rigid body to the world.
		addRigidBody(world, ci);

		// No longer need the reference on the sphereShape, as the rigidBody now owns it.
		sphereShape->removeReference();
	}

	position.add(gap);

	// Create and add a cylinder.
	{
		// Define a cylinder with a 1 metre radius and 2 metres long.
		hkVector4 start(0.0f, 0.0f, 0.0f);
		hkVector4 end(0.0f, 2.0f, 0.0f);
		hkReal radius = 1.0f;

		hkpCylinderShape* cylinderShape = new hkpCylinderShape(start, end, radius);

		// Set its properties.
		ci.m_shape = cylinderShape;
		ci.m_position = position;

		// Calculate the mass properties for the shape and add them to the rigid body's properties.
		hkMassProperties massProperties;
		hkInertiaTensorComputer::computeCylinderVolumeMassProperties(start, end, radius, mass, massProperties);
		ci.setMassProperties(massProperties);

		// Add the rigid body to the world.
		addRigidBody(world, ci);

		// No longer need the reference on the cylinderShape, as the rigidBody now owns it.
		cylinderShape->removeReference();
	}

	position.add(gap);

	// Create and add a capsule.
	{
		// Define a capsule with a 1 metre radius and 2 metres long.
		hkVector4 start(0.0f, 0.0f, 0.0f);
		hkVector4 end(0.0f, 2.0f, 0.0f);
		hkReal radius = 1.0f;

		hkpCapsuleShape* capsuleShape = new hkpCapsuleShape(start, end, radius);

		// Set its properties.
		ci.m_shape = capsuleShape;
		ci.m_position = position;

		// Calculate the mass properties for the shape and add them to the rigid body's properties.
		hkMassProperties massProperties;
		hkInertiaTensorComputer::computeCapsuleVolumeMassProperties(start, end, radius, mass, massProperties);
		ci.setMassProperties(massProperties);

		// Add the rigid body to the world.
		addRigidBody(world, ci);

		// No longer need the reference on the capsuleShape, as the rigidBody now owns it.
		capsuleShape->removeReference();
	}

	position.add(gap);

	// Create and add a triangle.
	{
		// Define the size of the triangle.
		hkVector4 v0(0.0f, 0.0f, 0.0f);
		hkVector4 v1(0.0f, 0.0f, 3.0f);
		hkVector4 v2(3.0f, 0.0f, 0.0f);

		hkpTriangleShape* triangleShape = new hkpTriangleShape(v0, v1, v2);
		//triangleShape->setExtrusion(hkVector4(0.0f, -0.5f, 0.0f));

		// Set its properties.
		ci.m_shape = triangleShape;
		ci.m_position = position;

		// Calculate the mass properties for the shape and add them to the rigid body's properties.
		hkMassProperties massProperties;
		const hkReal surfaceThickness = 10.0f;
		hkInertiaTensorComputer::computeTriangleSurfaceMassProperties(v0, v1, v2, mass, 1.0f, massProperties);
		ci.setMassProperties(massProperties);

		// Add the rigid body to the world.
		addRigidBody(world, ci);

		// No longer need the reference on the triangleShape, as the rigidBody now owns it.
		triangleShape->removeReference();
	}

	position.add(gap);

	// Create and add a convex shape with a number of specified vertices. These vertices do not necessarily need to
	// create a convex shape; Havok will generate a convex hull from them.
	{
		int numVertices = 4;

		// A hkReal is a float which has a size of 4 so with the 3 coordinates in a vertex plus its padding the stride
		// will equal 16.
		int stride = sizeof(hkReal) * 4;

		// Define the vertices of the shape.
		float vertices[] =
		{
			-2.0f, 2.0f, 1.0f, 0.0f,
			1.0f, 3.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 3.0f, 0.0f,
			1.0f, 0.0f, 0.0f, 0.0f
		};

		hkStridedVertices stridedVerts;
		stridedVerts.m_numVertices = numVertices;
		stridedVerts.m_striding = stride;
		stridedVerts.m_vertices = vertices;

		hkpConvexVerticesShape::BuildConfig config;

		hkpConvexVerticesShape* convexShape = new hkpConvexVerticesShape(stridedVerts, config);

		// Set its properties.
		ci.m_shape = convexShape;
		ci.m_position = position;

		hkpInertiaTensorComputer::setShapeVolumeMassProperties(convexShape, mass, ci);

		// Add the rigid body to the world.
		addRigidBody(world, ci);
		convexShape->removeReference();
	}
}

// Add a sphere that pushes through some boxes.
void motionTypes(hkpWorld* world)
{
	// Add a keyframed sphere.
	// As this sphere is keyframed, almost nothing will affect its motion and it will simply plow through the poor boxes.
	// It will still impart force on the boxes, pushing them aside.
	// Interestingly, keyframed bodies will go through fixed rigid bodies and bounce off other keyframed bodies.
	{
		// Define a sphere with a 1 metre radius.
		const hkReal radius = 1.0f;
		hkpSphereShape* sphereShape = new hkpSphereShape(radius);

		// Set its properties.
		hkpRigidBodyCinfo ci;
		ci.m_shape = sphereShape;
		ci.m_position = hkVector4(0.0f, 2.0f, 30.0f);
		ci.m_motionType = hkpMotion::MOTION_KEYFRAMED; // make the body keyframed, thus exclusively under our control.
		
		// Set the initial (and since it won't change - constant) velocity of the sphere.
		ci.m_linearVelocity = hkVector4(0.0f, 0.0f, -10.0f);

		addRigidBody(world, ci);

		// No longer need the reference on the sphereShape, as the rigidBody now owns it.
		sphereShape->removeReference();
	}

	// Add boxes for sphere to plow through.
	int numBoxes = 5;
	{
		hkVector4 halfExtents(1.0f, 1.0f, 1.0f);
		hkpBoxShape* boxShape = new hkpBoxShape(halfExtents);

		hkpRigidBodyCinfo ci;
		ci.m_shape = boxShape;
		ci.m_motionType = hkpMotion::MOTION_DYNAMIC;

		// Place the first box near the edge of the ground box, and rest it on the ground.
		// It is moved slightly in the x to make the boxes be pushed aside rather than pushed through.
		hkVector4 pos(halfExtents(1), 2.0f, -10.0f);

		// Add a slight gap between the boxes, along the Z.
		float gapZ = 3.0f + halfExtents(1) * 2;
		// Create the boxes.
		for (int i = 0; i < numBoxes; ++i)
		{
			pos.add(hkVector4(0.0f, 0.0f, gapZ));

			ci.m_position = pos;

			addRigidBody(world, ci);
		}

		boxShape->removeReference();
	}
}