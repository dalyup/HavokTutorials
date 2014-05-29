#pragma once

#ifndef HAVOK_CORE
#define HAVOK_CORE

#include <Common/Base/hkBase.h>

#include <Physics2012/Dynamics/World/hkpWorld.h>

#include <Common/Visualize/hkVisualDebugger.h>
#include <Physics2012/Utilities/VisualDebugger/hkpPhysicsContext.h>

class HavokCore
{
private:	
	hkVisualDebugger* _vdb;
	hkpPhysicsContext* _physicsContext;

	// The physics world.
	hkpWorld* _world;

	void initHavok();
	void deinitHavok();
public:
	HavokCore();
	~HavokCore();

	hkpWorld* getWorld() { return _world; }
	void stepSimulation(float deltaTime);
};

#endif