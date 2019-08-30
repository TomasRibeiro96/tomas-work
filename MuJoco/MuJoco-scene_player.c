// This file runs a simulation with the forces
// written on the .dat file by the multithread simulation in MuJoco

#include "/home/exs/.mujoco/mujoco200_linux/include/mujoco.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include <unistd.h>
#include "glfw3.h"
#include <stdbool.h>
#include </home/exs/glfw-3.3/include/GLFW/glfw3.h>

#define N 2
#define timestep 0.005    // timestep defined in xml file
#define total_sim_time 8  // Total simulation time in seconds

char error[1000];
mjModel* m = NULL;
mjData* d = NULL;
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods){
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE ){
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods){
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos){
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset){
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


int main(void){

    int k = 0;

	// activate MuJoCo
	mj_activate("/home/exs/.mujoco/mujoco200_linux/bin/mjkey.txt");

	FILE * fp;
	fp = fopen("/home/exs/.mujoco/mujoco200_linux/robot_arm/robot_arm-paral-v2.dat", "r");

	// load model from file and check for errors
	m = mj_loadXML("robot_arm.xml", NULL, error, 1000);
	if( !m ){
		printf("%s\n", error);
		return 1;
	}

	// make data corresponding to model
	d = mj_makeData(m);

	// init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

	// set initial camera position
	cam.lookat[0] = .6;
	cam.lookat[1] = 6;
	cam.lookat[0] = .8;
	cam.distance = 15;
	cam.azimuth = 0;
	cam.elevation = -30;

    mjtNum shoulder_force;
    mjtNum elbow_force;


    while( !glfwWindowShouldClose(window) ){

		while(d->time<total_sim_time){
			mj_step(m, d);

			fscanf(fp, "%lf", &shoulder_force);
            fscanf(fp, "%lf", &elbow_force);
			
			d->ctrl[0] = shoulder_force;
			d->ctrl[1] = elbow_force;

			// get framebuffer viewport
			mjrRect viewport = {0, 0, 0, 0};
			glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

			// update scene and render
			mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
			mjr_render(viewport, &scn, &con);

			// swap OpenGL buffers (blocking call due to v-sync)
			glfwSwapBuffers(window);

			// process pending GUI events, call GLFW callbacks
			glfwPollEvents();
		}

        if( k == 0 ){
            k++;
            printf("Box y position: %f\n", d->xpos[3*4+1]);
        }
	}

	fclose(fp);

	//free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

	// free model and data, deactivate
	mj_deleteData(d);
	mj_deleteModel(m);
	mj_deactivate();

	// terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

	return 0;
}