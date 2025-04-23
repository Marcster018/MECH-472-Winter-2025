
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

#include "update_simulation.h"

extern robot_system S1;

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double width1, height1;
	int n_robot;
	double x_obs[50] = { 0.0 }, y_obs[50] = { 0.0 };
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	int capture = 0;
	int v_mode;
	
	// number of obstacles
	const int N_obs = 2;
	
	// file names for all the obstacles
	// -- you must have the same number of names as obstacles
	// -- the names can be repeated though if the image is the same
	char obstacle_file[N_obs][S_MAX] = { 
		"obstacle_black.bmp" , "obstacle_green.bmp"
	};	
	
	// obstacle locations
	// -- you must set one location for each obstacle
	
	x_obs[0] = 270.5; // pixels
	y_obs[0] = 270.5; // pixels

	x_obs[1] = 135; // pixels
	y_obs[1] = 135; // pixels
	
	// TODO: it might be better to put this model initialization
	// section in a separate function
	
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1  = 640;
	height1 = 480;

	// set robot model parameters ////////
	
	D = 121.0; // distance between front wheels (pixels)
	
	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;
	
	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file
	
	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	activate_simulation(width1,height1,
		x_obs,y_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp",
		obstacle_file,D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);			
		
	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;
	
	// set simulation mode
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 0;
	set_simulation_mode(mode);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 0;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 375;
	theta0 = 3.14159/4;
	set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1250; // pulse width for left wheel servo (us)
	pw_r = 2000; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,max_speed);

	// opponent inputs
	pw_l_o = 1300; // pulse width for left wheel servo (us)
	pw_r_o = 1600; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
				opponent_max_speed);

	// regular vision program ////////////////////////////////
	
	// note that at this point you can write your vision program
	// exactly as before.
	
	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser.
	
	image rgb;
	int height, width;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width  = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	// allocate memory for the images
	allocate_image(rgb);

	// measure initial clock time
	tc0 = high_resolution_time(); 

	while(1) {
		
//		update_background();
//		update_obstacles();
			
		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

//		update_image(rgb);

		tc = high_resolution_time() - tc0;

		// fire laser
		if(tc > 1) laser = 1;
		
		if(tc > 9) laser_o = 1;

		if( (tc > 1.5) && !capture ) {
			save_rgb_image("output.bmp",rgb);
			capture = 1;
		}

		// turn off the lasers so we can fire it again later
		if(tc > 10) { 
			laser = 0;
			laser_o = 0;
		}
		
		// fire laser at tc = 14 s
		if(tc > 14) {
			laser = 1;
			
			// turn laser angle alpha at the same time
			pw_laser = 1000;
		}

		// change the inputs to move the robot around
		
		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels
		set_inputs(pw_l,pw_r,pw_laser,laser,max_speed);

		// manually set opponent inputs for the simulation
		// -- good for testing your program
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
					opponent_max_speed);

		// * v_mode is an optional argument for view_rgb_image(...)
		// - adjusting it might improve performance / reduce delays
		// -- see "image_transfer.h" for more details
		v_mode = 1;
		view_rgb_image(rgb,v_mode);
	}

	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}
