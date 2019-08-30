#include <ctype.h>
#include <iostream>
#include <omp.h>

#include <PxPhysicsAPI.h>
#include <PxVisualizationParameter.h>

#include "../snippetcommon/SnippetPrint.h"
#include "../snippetcommon/SnippetPVD.h"
#include "../snippetutils/SnippetUtils.h"

#include </home/exs/PhysX/physx/include/pvd/PxPvdSceneClient.h>

using namespace physx;

static const int num_threads = 100;
static const int num_timesteps = 200;						// 200 = 1 second simulation
static const PxReal target = 7.5;

////   A + B = 1   ////
static const PxReal A = .7;                       			// Weight of distance to target
static const PxReal B = .3;                       			// Weight of total force
static const PxReal noise_ratio = .05;

static PxRevoluteJoint* shoulder[num_threads] = {0};
static PxRevoluteJoint* elbow[num_threads] = {0};
static PxRigidDynamic* box_body[num_threads] = {0};
static PxRigidDynamic* base_body[num_threads] = {0};
static PxRigidDynamic* ua_body[num_threads] = {0};
static PxRigidDynamic* la_body[num_threads] = {0};

static const PxReal shoulder_kp = 1200;       							// Proportional factor for shoulder joint
static const PxReal shoulder_kd = 100;         							// Derivative factor for elbow joint
static const PxReal elbow_kp = 500;        	  							// Proportional factor for shoulder joint
static const PxReal elbow_kd = 10;										// Derivative factor for elbow joint

static const PxReal des_shoulder_angle = 1.57;     						// Desired position for shoulder
static const PxReal des_elbow_angle = 0;          						// Desired position for elbow

static PxReal shoulder_force[num_threads][num_timesteps];	// Force on shoulder joint
static PxReal elbow_force[num_threads][num_timesteps];		// Force on elbow joint

static PxReal distance_to_target[num_threads] = {0};
static PxReal total_force[num_threads] = {0};
static PxReal distance_to_target_max = 0;
static PxReal total_force_max = 0;
static PxReal sum_distance_to_target_diff = 0;
static PxReal sum_total_force_diff = 0;
static PxReal weight;
static PxReal final_shoulder_force[num_threads] = {0};
static PxReal final_elbow_force[num_threads] = {0};

const PxReal timestep = 0.005f;

static int i;
static int n;

////////////////////////////////////////////

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene[num_threads]		= {NULL};

PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd       = NULL;


int snippetMain(int, const char*const*){

	FILE * fp;
	fp = fopen("/home/exs/PhysX/physx/snippets/snippetmultithreading/joint_forces.dat", "w");


	omp_set_dynamic(0);                   // disable dynamic scheduling
	omp_set_num_threads(num_threads);

	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);
	PxInitExtensions(*gPhysics, gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	
	//PxU32 numCores = SnippetUtils::getNbPhysicalCores();
	gDispatcher = PxDefaultCpuDispatcherCreate(1);  // With smaller values the simulation is faster
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

	sceneDesc.solverType = PxSolverType::eTGS;

	sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	// gMaterial gives physical properties to material
	// Its arguments are:
	// - Static friction;
	// - Dynamic friction;
	// - Restitution coefficient.
	gMaterial = gPhysics->createMaterial(.3f, .3f, 0.f);

	#pragma omp parallel
	{
		// Variable declaration
		PxReal shoulder_angle, elbow_angle, shoulder_angle_error, elbow_angle_error;
		PxReal derv_shoulder_angle_error, derv_elbow_angle_error;
		PxReal prev_shoulder_angle_error, prev_elbow_angle_error;
		int thread_num = omp_get_thread_num();

		gScene[thread_num] = gPhysics->createScene(sceneDesc);
		PxPvdSceneClient* pvdClient = gScene[thread_num]->getScenePvdClient();

		if(pvdClient){
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
		}

		PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
		gScene[thread_num]->addActor(*groundPlane);


		////////// ROBOT ARM LINKS //////////
		// Base
		PxShape* base_shape = gPhysics->createShape(PxBoxGeometry(1, .25, 1), *gMaterial);
		base_body[thread_num] = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.f, .25f, 0.f), PxQuat(0, PxVec3(0.f, 0.f, 1.f))));
		base_body[thread_num]->setMass(50);
		base_body[thread_num]->attachShape(*base_shape);
		gScene[thread_num]->addActor(*base_body[thread_num]);

		// Upper Arm
		PxShape* ua_shape = gPhysics->createShape(PxCapsuleGeometry(.3, 1.485), *gMaterial);
		ua_body[thread_num] = gPhysics->createRigidDynamic( PxTransform( PxVec3(0, 1.9, 1), PxQuat(0.647, -0.286, -0.647, 0.286) ) );
		ua_body[thread_num]->setMass(10);
		ua_body[thread_num]->attachShape(*ua_shape);
		gScene[thread_num]->addActor(*ua_body[thread_num]);

		// Lower Arm
		PxShape* la_shape = gPhysics->createShape(PxCapsuleGeometry(.3, 1.25), *gMaterial);
		la_body[thread_num] = gPhysics->createRigidDynamic( PxTransform( PxVec3(0.6, 2, 2.75), PxQuat(0.632, 0.316, -0.632, -0.316) ) );
		la_body[thread_num]->setMass(10);
		la_body[thread_num]->attachShape(*la_shape);
		gScene[thread_num]->addActor(*la_body[thread_num]);


		////////// JOINTS //////////

		// Fixed base
		PxFixedJointCreate(*gPhysics, NULL, PxTransform(PxVec3(0, 0, 0)), base_body[thread_num], PxTransform(PxVec3(0, -.25, 0)));

		// Shoulder
		shoulder[thread_num] = PxRevoluteJointCreate( *gPhysics,
										base_body[thread_num], PxTransform( PxVec3(0, 0.55, 0), PxQuat(0.707, 0, 0, -0.707) ),
												ua_body[thread_num], PxTransform( PxVec3(1.486607, 0, 0), PxQuat(0.707, 0, -0.707, 0) ) );
		shoulder[thread_num]->setRevoluteJointFlags(PxRevoluteJointFlag::eDRIVE_ENABLED);
		shoulder[thread_num]->setDriveGearRatio(500);

		// Elbow
		elbow[thread_num] = PxRevoluteJointCreate( *gPhysics,
										ua_body[thread_num], PxTransform( PxVec3(-1.486607, 0, -0.6), PxQuat(0.707, 0, -0.707, 0) ),
												la_body[thread_num], PxTransform( PxVec3(1.25, 0, 0), PxQuat(0.707, 0, -0.707, 0) ) );
		elbow[thread_num]->setRevoluteJointFlags(PxRevoluteJointFlag::eDRIVE_ENABLED);
		elbow[thread_num]->setDriveGearRatio(500);


		////////// BOX //////////
		PxShape* box_shape = gPhysics->createShape(PxBoxGeometry(0.8, 0.8, 0.8), *gMaterial);
		box_body[thread_num] = gPhysics->createRigidDynamic( PxTransform( PxVec3(0.6, 0.8, 5)) );
		box_body[thread_num]->setMass(5);
		box_body[thread_num]->attachShape(*box_shape);
		gScene[thread_num]->addActor(*box_body[thread_num]);


		for(i=0; i<num_timesteps; i++){
			////// Gaussian random number genetarion between 0 and 1//////
			double a = (double)rand()/(double)RAND_MAX;
			double b = (double)rand()/(double)RAND_MAX;

			PxReal random1 = sqrt( -2*log(a) ) * cos( 2*M_PI*b );
			PxReal random2 = sqrt( -2*log(a) ) * sin( 2*M_PI*b );

			if(random1 < -1) random1 = -1;
			else if (random1 > 1) random1 = 1;
			
			if(random2 < -1) random2 = -1;
			else if (random2 > 1) random2 = 1;

			// Measure joint positions
			shoulder_angle = shoulder[thread_num]->getAngle();
			elbow_angle = elbow[thread_num]->getAngle();
			// Calculate errors
			shoulder_angle_error = des_shoulder_angle - shoulder_angle;
			elbow_angle_error = des_elbow_angle - elbow_angle;
			// Calculate errors' derivatives
			derv_shoulder_angle_error = (shoulder_angle_error - prev_shoulder_angle_error)/timestep;
			derv_elbow_angle_error = (elbow_angle_error - prev_elbow_angle_error)/timestep;
			// Calculate force for each joint
			shoulder_force[thread_num][i] = shoulder_angle_error*shoulder_kp
												+ derv_shoulder_angle_error*shoulder_kd;
			elbow_force[thread_num][i] = elbow_angle_error*elbow_kp
												+ derv_elbow_angle_error*elbow_kd;
			// Add noise
			shoulder_force[thread_num][i] += random1*noise_ratio*shoulder_force[thread_num][i];
			elbow_force[thread_num][i] += random2*noise_ratio*elbow_force[thread_num][i];
			// Set force on each joint
			shoulder[thread_num]->setDriveVelocity(shoulder_force[thread_num][i]);
			elbow[thread_num]->setDriveVelocity(elbow_force[thread_num][i]);
			// Save error
			prev_shoulder_angle_error = shoulder_angle_error;
			prev_elbow_angle_error = elbow_angle_error;
			// Save accumulated force
			total_force[thread_num] += shoulder_force[thread_num][i]
													+elbow_force[thread_num][i];
			// Save distance to target
			PxReal box_pos = box_body[thread_num]->getGlobalPose().p[2];
			distance_to_target[thread_num] = fabs(box_pos-target);

			// Advance simulation
			gScene[thread_num]->simulate(timestep);
			gScene[thread_num]->fetchResults(true);
		}

		gScene[thread_num]->release();
	}


	// Find maximum of distance_to_target and total_force
	for(n=0; n<num_threads; n++){
		if( distance_to_target[n] > distance_to_target_max ) 	distance_to_target_max = distance_to_target[n];
		if( total_force[n] > total_force_max ) 					total_force_max = total_force[n];
	}


	// Calculate denominators for weighted average
	for(n=0; n<num_threads; n++){
		sum_distance_to_target_diff += distance_to_target_max - distance_to_target[n];
		sum_total_force_diff += total_force_max - total_force[n];
	}


	// Average all forces and write to file
	for(i=0; i<num_timesteps; i++){
		for(int n=0; n<num_threads; n++){
			weight = A * ( distance_to_target_max - distance_to_target[n] ) / sum_distance_to_target_diff
									+ B * ( total_force_max - total_force[n] ) / sum_total_force_diff;

			final_shoulder_force[i] += shoulder_force[n][i]*weight;
			final_elbow_force[i] += elbow_force[n][i]*weight;
		}
		fprintf(fp, "%f \t %f\n", (double)final_shoulder_force[i], (double)final_elbow_force[i]);
	}

	gDispatcher->release();
	gPhysics->release();
	gPvd->release();
	transport->release();
	PxCloseExtensions();
	gFoundation->release();

	fclose(fp);

	return 0;
}
