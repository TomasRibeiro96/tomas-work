#include "/home/exs/.mujoco/mujoco200_linux/include/mujoco.h"
#include "stdio.h"
#include <stdlib.h>
#include "glfw3.h"
#include "math.h"
#include <time.h>
#include <omp.h>
#include <stdbool.h>


#define num_threads 1000
#define timestep 0.005               // timestep defined in xml file
#define total_sim_time 8             // Total simulation time in seconds

////   A + B = 1   ////
#define A 0                       // Weight of distance to target
#define B 1                       // Weight of total force
#define noise_ratio .05

#define shoulder_kp 50             // Proportional factor for shoulder joint
#define shoulder_kd 1              // Derivative factor for elbow joint
#define elbow_kp 50                // Proportional factor for shoulder joint
#define elbow_kd 1                 // Derivative factor for elbow joint
	
#define des_shoulder_angle -0.83   // Desired position for shoulder
#define des_elbow_angle 1.74       // Desired position for elbow

// num_timesteps = total_sim_time / timestep
#define num_timesteps 1600


char error[1000];
mjModel* m;
mjData* d[num_threads];
mjvOption* opt;


int main(void){

	FILE * fp;
	fp = fopen("/home/exs/.mujoco/mujoco200_linux/robot_arm/robot_arm-paral-v2.dat", "w");

	// activate MuJoCo
	mj_activate("/home/exs/.mujoco/mujoco200_linux/bin/mjkey.txt");

	// load model from file and check for errors
	m = mj_loadXML("robot_arm.xml", NULL, error, 1000);
	if( !m ){
		printf("%s\n", error);
		return 1;
	}

	omp_set_dynamic(0);                   // disable dynamic scheduling
	omp_set_num_threads(num_threads);
	
	int n;
	int i;

	// make data corresponding to model
	for(n=0; n<num_threads; n++)
		d[n] = mj_makeData(m);

	// getting body id
	int base_id = mj_name2id(m, mjOBJ_BODY, "base");
	int ua_id = mj_name2id(m, mjOBJ_BODY, "upper_arm");
	int la_id = mj_name2id(m, mjOBJ_BODY, "lower_arm");
	int box_id = mj_name2id(m, mjOBJ_BODY, "box");
	int target_id = mj_name2id(m, mjOBJ_BODY, "target");

	//// Variable declaration
	mjtNum total_force_max = 0;
	mjtNum distance_to_target_max = 0;
	mjtNum sum_distance_to_target_diff = 0;
	mjtNum sum_total_force_diff = 0;
	time_t seconds;
	seconds = time(NULL);
	srand(seconds);
	mjtNum weight;

	static mjtNum elbow_force[num_threads][num_timesteps]; // Force on elbow joint
	static mjtNum shoulder_force[num_threads][num_timesteps]; // Force on shoulder joint
	static mjtNum total_force[num_threads] = {0}; // Cost function (sum of forces in joints in all instants of time)
	static mjtNum distance_to_target[num_threads];   // Goal evaluation function (distance to target)
	static mjtNum final_shoulder_force[num_timesteps]; // Force to be applied on shoulder at each time instant
	static mjtNum final_elbow_force[num_timesteps]; // Force to be applied on elbow at each time instant


	/////////////////////////////////// Parallel section /////////////////////////////////////////////
	#pragma omp parallel
	{
		mjtNum shoulder_angle;       // Position of shoulder in radians
		mjtNum shoulder_angle_error;
		mjtNum prev_shoulder_angle_error;
		mjtNum derv_shoulder_angle_error;
		mjtNum elbow_angle;          // Position of elbow in radians
		mjtNum elbow_angle_error;
		mjtNum prev_elbow_angle_error;
		mjtNum derv_elbow_angle_error;

		int thread_num = omp_get_thread_num();
		int rand_max = RAND_MAX;
		int sim_num;
		mjtNum sim_time;
		
		// run simulation for sim_time seconds
		while(d[thread_num]->time<total_sim_time){

			mj_step(m, d[thread_num]);

			sim_time = d[thread_num]->time;
			sim_num = sim_time/timestep - 1;

			////// Random number genetarion //////

			mjtNum a = (mjtNum)rand()/(mjtNum)RAND_MAX;
			mjtNum b = (mjtNum)rand()/(mjtNum)RAND_MAX;

			mjtNum random1 = sqrt( -2*log(a) ) * cos( 2*M_PI*b );
			mjtNum random2 = sqrt( -2*log(a) ) * sin( 2*M_PI*b );

			if(random1 < -1) random1 = -1;
			else if (random1 > 1) random1 = 1;
			
			if(random2 < -1) random2 = -1;
			else if (random2 > 1) random2 = 1;

			// Measure joint positions
			shoulder_angle = d[thread_num]->sensordata[0];
			elbow_angle = d[thread_num]->sensordata[1];
			// Calculate errors
			shoulder_angle_error = des_shoulder_angle - shoulder_angle;
			elbow_angle_error = des_elbow_angle - elbow_angle;
			// Calculate errors' derivatives
			derv_shoulder_angle_error = (shoulder_angle_error - prev_shoulder_angle_error)/timestep;
			derv_elbow_angle_error = (elbow_angle_error - prev_elbow_angle_error)/timestep;
			// Calculate force for each joint
			shoulder_force[thread_num][sim_num] = shoulder_angle_error*shoulder_kp
													+ derv_shoulder_angle_error*shoulder_kd;
			elbow_force[thread_num][sim_num] = elbow_angle_error*elbow_kp
													+ derv_elbow_angle_error*elbow_kd;
			// Add noise
			shoulder_force[thread_num][sim_num] += random1*noise_ratio*shoulder_force[thread_num][sim_num];
			elbow_force[thread_num][sim_num] += random2*noise_ratio*elbow_force[thread_num][sim_num];
			// Set force on each joint
			d[thread_num]->ctrl[0] = shoulder_force[thread_num][sim_num];
			d[thread_num]->ctrl[1] = elbow_force[thread_num][sim_num];
			// Save error
			prev_shoulder_angle_error = shoulder_angle_error;
			prev_elbow_angle_error = elbow_angle_error;
			// Save accumulated force
			total_force[thread_num] += shoulder_force[thread_num][sim_num]
													+elbow_force[thread_num][sim_num];
			
			////// Distance to target //////
			mjtNum box_pos = d[thread_num]->xpos[3*box_id+1];
			mjtNum target_pos = d[thread_num]->xpos[3*target_id+1];
			distance_to_target[thread_num] = fabs(box_pos-target_pos);
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////


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

		for(n=0; n<num_threads; n++){

			weight = A * ( distance_to_target_max - distance_to_target[n] ) / sum_distance_to_target_diff
									+ B * ( total_force_max - total_force[n] ) / sum_total_force_diff;

			final_shoulder_force[i] += shoulder_force[n][i]*weight;
			final_elbow_force[i] += elbow_force[n][i]*weight;

		}
		fprintf(fp, "%f \t %f\n", final_shoulder_force[i], final_elbow_force[i]);
	}

	fclose(fp);

	// free model and data, deactivate
	for(n=0; n<num_threads; n++){
		mj_deleteData(d[n]);
	}
	mj_deleteModel(m);
	mj_deactivate();

	return 0;
}