// Loads forces written by PhysX-Simulation-multi_thread.cpp
// and runs and renders a simulation with them

#include <ctype.h>
#include <iostream>

#include <PxPhysicsAPI.h>
#include <PxVisualizationParameter.h>

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"

#include </home/exs/PhysX/physx/include/pvd/PxPvdSceneClient.h>

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;

FILE * fp = fopen("/home/exs/PhysX/physx/snippets/snippetmultithreading/joint_forces.dat", "r");

/////////////////////////////////////////////////////////////////////////////////////
PxRevoluteJoint* shoulder;
PxRevoluteJoint* elbow;
PxRigidDynamic* box_body;
PxRigidDynamic* base_body;
PxRigidDynamic* ua_body;
PxRigidDynamic* la_body;

static const PxU32 num_timesteps = 200;

double shoulder_force[num_timesteps];                 // Force on shoulder joint
double elbow_force[num_timesteps];                    // Force on elbow joint

const PxReal timestep = 0.005f;

/////////////////////////////////////////////////////////////////////////////////////


void initPhysics(bool /*interactive*/){
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	
	PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	sceneDesc.solverType = PxSolverType::eTGS;

	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient){
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}

	// gMaterial gives physical properties to material
	// Its arguments are:
	// - Static friction;
	// - Dynamic friction;
	// - Restitution coefficient.
	gMaterial = gPhysics->createMaterial(.3f, .3f, 0.f);

	PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
	gScene->addActor(*groundPlane);


	////////// ROBOT ARM LINKS //////////
	// Base
	PxShape* base_shape = gPhysics->createShape(PxBoxGeometry(1, .25, 1), *gMaterial);
	base_shape->setFlag(PxShapeFlag::eVISUALIZATION, 2.0f);
	base_body = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.f, .25f, 0.f), PxQuat(0, PxVec3(0.f, 0.f, 1.f))));
	base_body->setMass(50);
	base_body->attachShape(*base_shape);
	gScene->addActor(*base_body);

	// Upper Arm
	PxShape* ua_shape = gPhysics->createShape(PxCapsuleGeometry(.3, 1.485), *gMaterial);
	ua_shape->setFlag(PxShapeFlag::eVISUALIZATION, 2.0f);
	ua_body = gPhysics->createRigidDynamic( PxTransform( PxVec3(0, 1.9, 1), PxQuat(0.647, -0.286, -0.647, 0.286) ) );
	ua_body->setMass(10);
	ua_body->attachShape(*ua_shape);
	gScene->addActor(*ua_body);

	// Lower Arm
	PxShape* la_shape = gPhysics->createShape(PxCapsuleGeometry(.3, 1.25), *gMaterial);
	la_shape->setFlag(PxShapeFlag::eVISUALIZATION, 2.0f);
	la_body = gPhysics->createRigidDynamic( PxTransform( PxVec3(0.6, 2, 2.75), PxQuat(0.632, 0.316, -0.632, -0.316) ) );
	la_body->setMass(10);
	la_body->attachShape(*la_shape);
	gScene->addActor(*la_body);


	////////// JOINTS //////////

	// Fixed base
	PxFixedJointCreate(*gPhysics, NULL, PxTransform(PxVec3(0, 0, 0)), base_body, PxTransform(PxVec3(0, -.25, 0)));

	// Shoulder
	shoulder = PxRevoluteJointCreate( *gPhysics,
									base_body, PxTransform( PxVec3(0, 0.55, 0), PxQuat(0.707, 0, 0, -0.707) ),
											ua_body, PxTransform( PxVec3(1.486607, 0, 0), PxQuat(0.707, 0, -0.707, 0) ) );
	shoulder->setRevoluteJointFlags(PxRevoluteJointFlag::eDRIVE_ENABLED);
	shoulder->setDriveGearRatio(500);

	// Elbow
	elbow = PxRevoluteJointCreate( *gPhysics,
									ua_body, PxTransform( PxVec3(-1.486607, 0, -0.6), PxQuat(0.707, 0, -0.707, 0) ),
											la_body, PxTransform( PxVec3(1.25, 0, 0), PxQuat(0.707, 0, -0.707, 0) ) );
	elbow->setRevoluteJointFlags(PxRevoluteJointFlag::eDRIVE_ENABLED);
	elbow->setDriveGearRatio(500);


	////////// BOX //////////
	PxShape* box_shape = gPhysics->createShape(PxBoxGeometry(0.8, 0.8, 0.8), *gMaterial);
	box_body = gPhysics->createRigidDynamic( PxTransform( PxVec3(0.6, 0.8, 5)) );
	box_body->setMass(5);
	box_body->attachShape(*box_shape);
	gScene->addActor(*box_body);


	//// Constrain movement of box in one direction (z)
	// The reference frame for actor 0 (worldbody) is turned 90 degrees so its x axis
	// points in the direction of the movement
	PxPrismaticJointCreate(*gPhysics,
									NULL, PxTransform(PxVec3(0.6, 0.8, 5), PxQuat(0.707, 0, 0.707, 0)),
											box_body, PxTransform(PxVec3(0, 0, 0), PxQuat(0.707, 0, -0.707, 0)));
}


void stepPhysics(bool /*interactive*/){
	fscanf(fp, "%lf", &shoulder_force[k]);
	fscanf(fp, "%lf", &elbow_force[k]);

	shoulder->setDriveVelocity(shoulder_force[k]);
	elbow->setDriveVelocity(elbow_force[k]);

	gScene->simulate(timestep);
	gScene->fetchResults(true);
}


void cleanupPhysics(bool /*interactive*/){	
	gScene->release();
	gDispatcher->release();
	gPhysics->release();	
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();
	PxCloseExtensions();  
	gFoundation->release();
	fclose(fp);


	//printf("robot_arm done.\n");
}


void keyPress(unsigned char /*key*/, const PxTransform& /*camera*/)
{
}


int snippetMain(int, const char*const*){
	

	#ifdef RENDER_SNIPPET
		extern void renderLoop();
		renderLoop();
	#else
		initPhysics(false);
		for(PxU32 i=0; i<num_timesteps; i++)
			stepPhysics(false);
		cleanupPhysics(false);
	#endif

	fclose(fp);

	return 0;
}
