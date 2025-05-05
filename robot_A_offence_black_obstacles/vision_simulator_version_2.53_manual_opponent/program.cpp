
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include <Windows.h>

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

extern robot_system S1;
void find_hollow_circles(int& nlabels, image& rgb, image& label, image& a, image& rgb0, int Ic[4], int Jc[4]) {
	ibyte* p, * pc;
	i2byte* pl, * plc;

	int R, G, B;

	pl = (i2byte*)label.pdata;

	int i, j, width, height;
	double ic, jc;
	int kc;

	width = rgb.width;
	height = rgb.height;

	p = rgb0.pdata;

	//go through all labels and check if there centroid are on a backgound
	for (int label_num = 1; label_num <= nlabels; label_num++) {

		centroid(a, label, label_num, ic, jc);
		kc = (int)ic + (int)jc * width;

		plc = pl + kc;
		pc = p + 3 * kc;

		B = *pc;
		G = *(pc + 1);
		R = *(pc + 2);

		//if centroid on a backgound check the color on the orignal image to associate properly in vector Ic,Jc
		if (*plc == 0) {

			//green
			if (R < 120 && G >120 && B < 150) {
				draw_point_rgb(rgb, ic, jc, 0, 255, 0);
				Ic[0] = ic;
				Jc[0] = jc;
			}
			//Red
			else if (R > 100 && G < 100 && B < 100) {
				draw_point_rgb(rgb, ic, jc, 255, 0, 0);
				Ic[1] = ic;
				Jc[1] = jc;
			}
			//orange
			else if (R > 220 && G > 130 && G < 200 && B < 150) {
				draw_point_rgb(rgb, ic, jc, 255, 100, 0);
				Ic[2] = ic;
				Jc[2] = jc;
			}
			//blue
			else if (R < 60 && G < 175 && B > 200) {
				draw_point_rgb(rgb, ic, jc, 0, 0, 255);
				Ic[3] = ic;
				Jc[3] = jc;
			}
		}
	}

}

void clean_up(image& rgb, image& a) {
	int width, height;
	image b;
	ibyte* p;

	width = rgb.width;
	height = rgb.height;

	b.type = GREY_IMAGE;
	b.width = width;
	b.height = height;

	allocate_image(b);
	
	copy(rgb, a);
	threshold(a, b,	70);
	copy(b, a);
	lowpass_filter(a, b);
	copy(b, a);
	invert(a, b);
	copy(b, a);
	erode(a, b);
	copy(b, a);
	dialate(a, b);
	copy(b, a);
	dialate(a, b);
	copy(b, a);
	copy(a, rgb);

	free_image(b);
}

void find_obstacles(image& rgb, image& label, image& a, int nlabel, vector<int> &OL) {
	//the function calculates the radius in 8 directions (every 45 degrees) and checks the diff in the
	//min and max value for each label
	int i, j, k, l, height, width, size;
	double ic, jc;
	i2byte* pl;

	pl = (i2byte*)label.pdata;
	height = rgb.height;
	width = rgb.width;
	size = width * height;

	//different directions
	int dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
	int dy[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };

	for (l = 1; l <= nlabel; l++) {
		centroid(a, label, l, ic, jc); //find centroid 
		float R[8] = { 0 };//make Radius vector all 0s

		for (int d = 0; d < 8; d++) {//from the centroid go in all 8 directions
			i = ic;
			j = jc;
			while (true) { //calculates the number of pixels that arent backgrounds in a specific direction
				i += dx[d];
				j += dy[d];
				if (i < 0 || j < 0 || i >= width || j >= height) break; //if pixels is out of bounds

				k = i + j * width;
				if (pl[k] != l) break; //if pixel doesnt match the correct label

				float x = i - ic;
				float y = j - jc;
				R[d] = sqrt(x * x + y * y); //finds the magnitude of the x and y direction
			}
		}

		float min = R[0], max = R[0]; //sets initial values
		for (int d = 1; d < 8; d++) { //checks each value in array to find max and min
			if (R[d] < min) min = R[d];
			if (R[d] > max) max = R[d];
		}

		float diff = max - min;
		if (diff < 3 && diff>0) {//by trial and error below 3 were the obstacles for diff
			OL.push_back(l);
			centroid(a, label, l, ic, jc);
			draw_point_rgb(rgb, ic, jc, 255, 0, 0);
		}
	}
}

void opponent_track(int Ic[4], int Jc[4], image& rgb, image& label, int& pw_r, int& pw_l) {

	//center of opponent
	int id = (Ic[3] + Ic[2]) / 2;
	int jd = (Jc[3] + Jc[2]) / 2;

	int width = rgb.width;
	int height = rgb.height;
	int size = width * height;

	int k, i, j;
	i2byte* pl, * plc;

	pl = (i2byte*)label.pdata;


	//drawing line between red point and enemy
	//line is y=mx+b

	int dx = (Ic[0] - id);
	int dy = (Jc[0] - jd);

	float m, b, y;

	if (dx == 0) {
		m = 0;
		b = Ic[0];
	}
	else {
		m = (float)dy / (float)dx;
		b = Jc[0] - Ic[0] * m;
	}

	//want to check if the red dot Ic[1], Jc[1] is bellow or above the line than turn the robot accordingly
	float deadzone = 10;
	y = m * Ic[1] + b;

	if (id <= Ic[0]) {
		if (Jc[1] < (y - deadzone)) {
			pw_r = 2000;
			pw_l = 2000;
		}
		else if (Jc[1] > (y + deadzone)) {
			pw_r = 1000;
			pw_l = 1000;
		}
		else {
			pw_r = 1500;
			pw_l = 1500;
		}
	}

	else if (id > Ic[0]) {
		if (Jc[1] < (y - deadzone)) {
			pw_r = 1000;
			pw_l = 1000;
		}
		else if (Jc[1] > (y + deadzone)) {
			pw_r = 2000;
			pw_l = 2000;
		}
		else {
			pw_r = 1500;
			pw_l = 1500;
		}
	}

	//drawing point
	draw_point_rgb(rgb, id, jd, 255, 0, 255); //draw point of where the robot looks

}

void detect_obstruction(int Ic[4], int Jc[4], vector<int>& OL, image& rgb, image& label, bool& detected) {
	int width = rgb.width;
	int height = rgb.height;
	int size = width * height;
	bool obstruction = false; //different bool so it is only updated at the end

	// Pointer to label data
	i2byte* pl = (i2byte*)label.pdata;

	// Center of defending
	int cd_x = (Ic[2] + Ic[3]) / 2;
	int cd_y = (Jc[2] + Jc[3]) / 2;

	// Center of rotation attacking
	int cr_x = Ic[0];
	int cr_y = Jc[0];

	// Calculate line equation parameters y = mx + b
	float dx = cr_x - cd_x;
	float dy = cr_y - cd_y;

	if (dx == 0) {
		for (int k = 0; k < size; k++) {
			int i = k % width;
			int j = (k - i) / width;
			if (abs(i - (int)cr_x) <= 1) {
				draw_point_rgb(rgb, i, j, 255, 0, 0);
				for (int a : OL) {
					if (pl[k] == a) obstruction = true;
				}
			}
		}
	}
	else {
		float m = dy / dx;
		float b = cr_y - cr_x * m;
		//line drawing

		//check if the pixels on the line have the same label as the obstacles
		for (int k = 0; k < size; k++) {
			int i = k % width;
			int j = (k - i) / width;
			float y = i * m + b;
			if (abs(j - (int)y) <= 1) {
				draw_point_rgb(rgb, i, j, 255, 0, 0);
				for (int a : OL) {
					if (pl[k] == a) obstruction = true;
				}
			}
		}
	}
	detected = obstruction;
}

void go_to(int Ic[4], int Jc[4], int& pw_l, int& pw_r, image& rgb, image& label, int id, int jd) {
	int i, j, k, width, size, height;
	width = rgb.width;
	height = rgb.height;
	size = width * height;
	bool locked = false;
	i2byte* pl;
	pl = (i2byte*)label.pdata;
	k = id + jd * width;

	draw_point_rgb(rgb, id, jd, 255, 0, 0);

	float angle_of_robot = atan2(Jc[0] - Jc[1], Ic[0] - Ic[1]);
	float V[3]; //0-x, 1-y, 2-angle
	V[0] = (Ic[0] - id);
	V[1] = (Jc[0] - jd);
	V[2] = atan2(jd - Jc[0], id - Ic[0]);
	float angle_error =V[2] - angle_of_robot;
	if (angle_error<-0.1) {
		pw_l = 1000;
		pw_r = 1000;
	}
	else if (angle_error>0.1) {
		pw_l = 2000;
		pw_r = 2000;
	}
	else {
		float tol = 10;
		if (fabs(V[1]) > tol || fabs(V[0]) > tol) {
			pw_l = 1000;
			pw_r = 2000;
		}
		else {
			pw_l = 1500;
			pw_r = 1500;
		}
	}
}

void quadrant(int Ic[4], int Jc[4], image& rgb, int& id, int& jd) {
	int width = rgb.width;
	int height = rgb.height;

	int cx = width / 2;
	int cy = height / 2;

	// Opponent's centroid
	int rx = (Ic[2] + Ic[3]) / 2;
	int ry = (Jc[2] + Jc[3]) / 2;

	// Representative positions for each quadrant (Q1 to Q4)
	int Qx[4] = { 3 * cx / 2,     cx / 2,     cx / 2,    3 * cx / 2 };
	int Qy[4] = { 3 * cy / 2, 3 * cy / 2,     cy / 2,        cy / 2 };

	if (rx > cx && ry > cy) {        // Q1 (bottom-right)
		id = Qx[2];  // move to Q3 (top-left)
		jd = Qy[2];
	}
	else if (rx < cx && ry > cy) {   // Q2 (bottom-left)
		id = Qx[3];  // move to Q4 (top-right)
		jd = Qy[3];
	}
	else if (rx < cx && ry < cy) {   // Q3 (top-left)
		id = Qx[0];  // move to Q1 (bottom-right)
		jd = Qy[0];
	}
	else if (rx > cx && ry < cy) {   // Q4 (top-right)
		id = Qx[1];  // move to Q2 (bottom-left)
		jd = Qy[1];
	}

}

void Collision_Detection(robot* my_robot, image& label, int& pw_l, int& pw_r) {
	const int width = label.width;
	const int height = label.height;

	// Define a scan zone in front of the robot
	const int safe_distance_px = 30;
	const int scan_width = 5;

	// Get robot's current position and heading
	double theta = my_robot->x[1];
	double rx = my_robot->x[2];
	double ry = my_robot->x[3];

	i2byte* pl = (i2byte*)label.pdata;

	// project forward points in a fan shape
	// check within range of 10 to 30 pixels if there are any objects in the way
	for (int r = 10; r <= safe_distance_px; r += 2) {
		for (int offset = -scan_width; offset <= scan_width; offset++) {
			double angle = theta + offset * 0.05;
			int i = (int)(rx + r * cos(angle));
			int j = (int)(ry + r * sin(angle));

			//check for screen boundaries
			if (i < 0 || i >= width || j < 0 || j >= height) {
				// Near screen edge: back up and turn slightly
				pw_l = 1300;
				pw_r = 1700;
				return;
			}

			// check for objectts
			int label_val = pl[j * width + i];
			if (label_val != 0) {
				// Something is directly ahead!
				cout << "\n[Vision Collision] Label detected at (" << i << ", " << j << ") = " << label_val;

				int delta_l = abs(pw_l - 1500);
				int delta_r = abs(pw_r - 1500);

				if (((delta_l > delta_r) && (pw_l > pw_r)) || ((delta_l < delta_r) && (pw_l < pw_r))) {
					// turning left -> turn harder left
					pw_l = 2000; // backwards left
					pw_r = 2000; // forward right
				}
				else if (((delta_l > delta_r) && (pw_l < pw_r)) || ((delta_l < delta_r) && (pw_l > pw_r))) {
					// turning right -> turn harder right
					pw_l = 1000; // forward left
					pw_r = 1000; // backwards right
				}
				else if ((delta_l == delta_r) && (pw_l != pw_r) && (pw_l < 1500 && pw_r > 1500)) {
					// if velocity was forward -> turns backwards
					pw_l = 2000; // backwards left
					pw_r = 1000; // backwards right 
				}
				return;
			}
		}
	}
}

void attack(image& rgb,image&rgb0, image&a, image& label, int &pw_r, int &pw_l) {
	int nlabel, Ic[4], Jc[4];
	vector<int> OL;

	copy(rgb, rgb0);
	clean_up(rgb, a);
	label_image(a, label, nlabel);
	bool detected = false;
	find_hollow_circles(nlabel, rgb, label, a, rgb0, Ic, Jc);
	find_obstacles(rgb, label, a, nlabel, OL);
	detect_obstruction(Ic, Jc, OL, rgb, label, detected);
	if (detected == false) opponent_track(Ic, Jc, rgb, label, pw_r, pw_l);
	else {
		int id, jd;
		quadrant(Ic, Jc, rgb, id, jd);
		go_to(Ic, Jc, pw_l, pw_r, rgb, label,id,jd);
	}
	cout << "\n" << detected;
	
}

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l=0, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	
	// TODO: it might be better to put this model initialization
	// section in a separate function
	
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1  = 640;
	height1 = 480;
	
	// number of obstacles
	const int N_obs  = 2;

	x_obs[0] = 270; // pixels
	y_obs[0] = 270; // pixels
	

	x_obs[1] = 270; // pixels
	y_obs[1] = 100; // pixels
	

	// set robot model parameters ////////
	
	D = 121.0; // distance between front wheels (pixels)
	
	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37;
	Ay = 0.0;
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;


	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file
	
	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	char obstacle_file[N_obs][S_MAX] = {
		"obstacle_black.bmp","obstacle_black.bmp"
	};

	activate_simulation(width1, height1, x_obs, y_obs, N_obs, "robot_A.bmp", "robot_B.bmp", "background.bmp",
		obstacle_file, D, Lx, Ly, Ax, Ay, alpha_max, n_robot);

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;
	
	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 0;
	level = 1;
	set_simulation_mode(mode);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 3.14159/4;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 375;
	theta0 = 3.14159/4;
	set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)
	
	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)

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
	
	image rgb, label, rgb0, a, b;
	int height, width, nlabel, Ic[4], Jc[4], Oc[2], Obs[2][2], ObsLabel[2];

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width  = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	label.type = LABEL_IMAGE;
	label.width = width;
	label.height = height;

	rgb0.type = RGB_IMAGE;
	rgb0.width = width;
	rgb0.height = height;

	a.type = GREY_IMAGE;
	a.width = width;
	a.height = height;

	b.type = GREY_IMAGE;
	b.width = width;
	b.height = height;

	// allocate memory for the images
	allocate_image(rgb);
	allocate_image(label);
	allocate_image(rgb0);
	allocate_image(a);
	allocate_image(b);

	// measure initial clock time
	tc0 = high_resolution_time(); 

	int init = 0;

	double angle = 0; double angle_op = 0;

	while(1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;

		// fire laser
		if(tc > 1) laser = 1;
		
		if(tc > 9) laser_o = 1;

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
		// or change some additional parameters (lighting, etc.)
		
		// only the following inputs work so far
		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels

		set_inputs(pw_l,pw_r,pw_laser,laser,
			max_speed);

		if (KEY(VK_UP)) {
			pw_l_o = 1000;
			pw_r_o = 2000;
		}
		else if (KEY(VK_DOWN)) {
			pw_l_o = 2000;
			pw_r_o = 1000;
		}
		else if (KEY(VK_LEFT)) {
			pw_l_o = 2000;
			pw_r_o = 2000;
		}
		else if (KEY(VK_RIGHT)) {
			pw_l_o = 1000;
			pw_r_o = 1000;
		}
		else {
			pw_l_o = 1500;
			pw_r_o = 1500;
		}

		// manually set opponent inputs for the simulation
		// -- good for testing your program
		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
					opponent_max_speed);

		//setting up for centroid calculation
	
		attack(rgb,rgb0,a, label, pw_r, pw_l);
		
		view_rgb_image(rgb);	

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
	}


	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}
