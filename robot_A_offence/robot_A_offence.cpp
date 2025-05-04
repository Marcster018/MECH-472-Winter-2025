
//Standard libraries
#include <cstdio>
#include <iostream>
#include <fstream>
#include <Windows.h>

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

//MECH 472 Libraries
#include "image_transfer.h"
#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"
#include "update_simulation.h"

//Our team's functions
#include "Team_functions.h"
#include "Example.h"

extern robot_system S1;

int main()
{

	//Define Local Variables
	double x0, y0, theta0, x0_o, y0_o, theta0_o, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double width1, height1;
	int n_robot;
	double x_obs[50] = { 0.0 }, y_obs[50] = { 0.0 };
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int opponent_mode;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	int capture = 0;
	int v_mode;	

	//Initialize Local Variables
	v_mode = 1;
	opponent_mode = 0;
	
	//Obstacle Variables
	const int N_obs = 5;	// number of obstacles
	char obstacle_file[N_obs][S_MAX] = { //File names for all the obstacles. You must have the same number of names as obstacles
		"obstacle_blue.bmp" , "obstacle_green.bmp", "obstacle_orange.bmp", "obstacle_red.bmp", "obstacle_black.bmp"
	};		
	x_obs[0] = 300; // pixels
	y_obs[0] = 135; // pixels

	x_obs[1] = 300; // pixels
	y_obs[1] = 400; // pixels

	x_obs[2] = 185; // pixels
	y_obs[2] = 135; // pixels

	x_obs[3] = 185; // pixels
	y_obs[3] = 400; // pixels

	x_obs[4] = 450; // pixels
	y_obs[4] = 250; // pixels


	//Map Variables
	width1  = 640;
	height1 = 480;

	//Robot Variables
	D = 121.0; // distance between front wheels (pixels)
	Lx = 31.0;// Position of laser in local robot coordinates (pixels) (Locally, robot is pointing in x)	
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	n_robot = 2;// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	
	cout << "\npress space key to begin program.";
	pause();

	//Setup Simulation

	activate_vision();
	activate_simulation(width1,height1,x_obs,y_obs,N_obs,"robot_A.bmp","robot_B.bmp","background.bmp",obstacle_file,D,Lx,Ly,Ax,Ay,alpha_max,n_robot);			

	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 0;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
	x0_o = 75;
	y0_o = 150;
	theta0_o = 1.5708;
	set_opponent_position(x0_o,y0_o,theta0_o);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs (No initial movements)
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,max_speed);

	// opponent inputs (No initial movement)
	pw_l_o = 1500; // pulse width for left wheel servo (us)
	pw_r_o = 1500; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
				opponent_max_speed);
	

	//Gameplay

	//Define Gameplay variables
	image rgb;
	int height, width;
	char Player;

	//Initialize Gameplay variables
	width  = 640;
	height = 480;
	Player = 'A';

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	// allocate memory for the images
	allocate_image(rgb);
	acquire_image_sim(rgb);
	// measure initial clock time
	tc0 = high_resolution_time(); 

	//Determine opponent behaviour
	cout << "\nOpponent behaviour has been predefined to best illustrate the capabilities of our Attack program.\nHowever the opponent can also be controlled using keyboard input for further testing.";
	cout << "\n\nTo run the program against the enemy with the predefined behaviour, press 1.";
	cout << "\n\nTo run the program against the enemy with behaviour controlled by a keyboard, press 2.";
	cout << "\n\nOnce an opponent is selected, press 'Enter' to begin\n";

	while (true) {
		cin >> opponent_mode;

		if (opponent_mode == 1 || opponent_mode == 2) {
			break;
		}
		else {
			cout << "\n\n This input is not valide. Please enter 1 or 2";
				opponent_mode = 0;
		}
	}

	while(1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);
		tc = high_resolution_time() - tc0;

		//Control our robot
		Attack_Sequence(rgb, pw_l, pw_r, pw_laser, laser, Player);
		set_inputs(pw_l,pw_r,pw_laser,laser,max_speed);

		//Control opponent robot

		if (opponent_mode == 1) {
			//This function uses variables directly from the simulator since it is only used as an example to show the behaviour of our attack function
			Example_Defence(rgb, pw_l_o, pw_r_o);
		}
		else if (opponent_mode == 2) {
			Example_KeyboardInput(pw_l_o, pw_r_o);
		}
		else {
			cout << "There was an error in determining the behaviour of the oppponent.";
			break;
		}

		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, opponent_max_speed);

		//View changes
		view_rgb_image(rgb,v_mode);
		pause();
	}

	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}
