// The majority of the code for this class and explanations for what some of it does is taken from two sources.
// The 'Creating a simple game using Havok Physics' tutorial found here: https://software.intel.com/en-us/forums/topic/278673
// The Physics2012Example is found in <havokversion>/Demo/StandAloneDemos/Physics2012

#include "HavokCore.h"

#include <iostream>

#include <Common/Base/Memory/System/Util/hkMemoryInitUtil.h>
#include <Common/Base/Memory/Allocator/Malloc/hkMallocAllocator.h>
#include <Physics2012/Collide/Dispatch/hkpAgentRegisterUtil.h>

static void HK_CALL errorReport(const char* msg, void* userContext)
{
	std::cerr << msg << std::endl;
}

HavokCore::HavokCore()
{
	initHavok();
}

HavokCore::~HavokCore()
{
	deinitHavok();
}

void HavokCore::initHavok()
{
	// Since we are debugging (thus using the VDB) we will use the checking memory system. Although less efficient than the default one,
	// it performs analysis on the memory used and tells us if any Havok memory leaked during execution.
	hkMemoryRouter* memoryRouter = hkMemoryInitUtil::initChecking(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024*1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	// Set up the physics world's information.
	hkpWorldCinfo worldInfo;
	worldInfo.m_gravity = hkVector4(0.0f, -9.8f, 0.0f); // just set to the value that gravity actually is

	// Flag objects that fall out of the simulated world's border to be automatically removed, and set the size of that border.
	worldInfo.m_broadPhaseBorderBehaviour = hkpWorldCinfo::BROADPHASE_BORDER_REMOVE_ENTITY;
	worldInfo.setBroadPhaseWorldSize(1000.0f);

	// Create the physics world with the information.
	_world = new hkpWorld(worldInfo);

	// Register all the collision agents so collisions can be detected.
	hkpAgentRegisterUtil::registerAllAgents(_world->getCollisionDispatcher());

	//
	// Initialise the visual debugger.
	// The context must exist beyond the use of the VDB instance, and you can make
	// whatever contexts you like for your own viewer types.
	//

	_physicsContext = new hkpPhysicsContext();
	hkpPhysicsContext::registerAllPhysicsProcesses(); // all the physics viewers
	_physicsContext->addWorld(_world); // add the physics world so the viewers can see it
	hkArray<hkProcessContext*> contexts;
	contexts.pushBack(_physicsContext);
	_vdb = new hkVisualDebugger(contexts);
	_vdb->serve();
}

void HavokCore::deinitHavok()
{
	// Clean up the visual debugger and the contexts.
	// Contexts are not reference counted at the base class level by the VDB as
	// they are just interfaces really. So only delete the context after you have
	// finished using the VDB.
	_vdb->removeReference();
	_physicsContext->removeReference();

	// Clean up the physics world.
	_world->removeReference();

	// Quit the base and memory system.
	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
}

void HavokCore::stepSimulation(float deltaTime)
{
	_world->stepDeltaTime(deltaTime);

	_vdb->step();
}

#include <Common/Base/keycode.cxx>

// This excludes libraries that are not going to be linked from the project configuration, even if the keycodes are
// present.
#undef HK_FEATURE_PRODUCT_AI
#undef HK_FEATURE_PRODUCT_ANIMATION
#undef HK_FEATURE_PRODUCT_CLOTH
#undef HK_FEATURE_PRODUCT_DESTRUCTION
#undef HK_FEATURE_PRODUCT_DESTRUCTION_2012
#undef HK_FEATURE_PRODUCT_BEHAVIOUR
#undef HK_FEATURE_PRODUCT_MILSIM
#undef HK_FEATURE_PRODUCT_PHYSICS

#define HK_EXCLUDE_LIBRARY_hkpVehicle
#define HK_EXCLUDE_LIBRARY_hkCompat
#define HK_EXCLUDE_LIBRARY_hkSceneData
#define HK_EXCLUDE_LIBRARY_hkcdCollide

//
// Common
//
#define HK_EXCLUDE_FEATURE_SerializeDeprecatedPre700
#define HK_EXCLUDE_FEATURE_RegisterVersionPatches 

//
// Physics
//
#define HK_EXCLUDE_FEATURE_hkpHeightField

#define HK_EXCLUDE_FEATURE_hkpAccurateInertiaTensorComputer

#define HK_EXCLUDE_FEATURE_hkpUtilities
#define HK_EXCLUDE_FEATURE_hkpVehicle
#define HK_EXCLUDE_FEATURE_hkpCompressedMeshShape
#define HK_EXCLUDE_FEATURE_hkpConvexPieceMeshShape
#define HK_EXCLUDE_FEATURE_hkpExtendedMeshShape
#define HK_EXCLUDE_FEATURE_hkpMeshShape
#define HK_EXCLUDE_FEATURE_hkpSimpleMeshShape
#define HK_EXCLUDE_FEATURE_hkpPoweredChainData
#define HK_EXCLUDE_FEATURE_hkMonitorStream

#include <Common/Base/Config/hkProductFeatures.cxx>