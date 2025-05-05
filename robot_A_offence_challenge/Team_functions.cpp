
//Our Team's Functions

//Standard Functions
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <cstdio>

//MECH 472 Libraries
#include "image_transfer.h"
#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"

#include "Team_functions.h"

using namespace std;
#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

//Attack Function
void Attack_Sequence(image& rgb, int& pw_r, int& pw_l, int& pw_laser, int& laser, char Player) {

	int height, width, nlabelBW, nlabelColour;
	static vector <array <int, 6>> Bulk_Data; //Using vector to be able to scale the number of rows to the number of objects
	static array<array<int, 6>, 2> Robot_Data; //Using array notation to be able to use the function of array a = array b instead of using pointer notation
	static array<array<int, 6>, 2> Opponent_Data;//Using array notation to be able to use the function of array a = array b instead of using pointer notation
	static vector <array <int, 6>> Obstacle_Data;//Using vector to be able to scale the number of rows to the number of objects
	bool detected = false;
	image ProcessingImage, LabelImageBW, LabelImageColour,a;

	ProcessingImage.type = RGB_IMAGE;
	ProcessingImage.width = rgb.width;
	ProcessingImage.height = rgb.height;

	LabelImageBW.type = LABEL_IMAGE;
	LabelImageBW.width = rgb.width;
	LabelImageBW.height = rgb.height;

	LabelImageColour.type = LABEL_IMAGE;
	LabelImageColour.width = rgb.width;
	LabelImageColour.height = rgb.height;

	a.type = GREY_IMAGE;
	a.width = rgb.width;
	a.height = rgb.height;

	allocate_image(LabelImageBW);
	allocate_image(LabelImageColour);
	allocate_image(ProcessingImage);
	allocate_image(a);

	copy(rgb, ProcessingImage);

	//Converting to Greyscale
	Process_Image(ProcessingImage, LabelImageBW, LabelImageColour, nlabelBW, nlabelColour);

	//Converting to Label Image
	Get_Image_Data(rgb,LabelImageBW,LabelImageColour, nlabelBW,nlabelColour,Bulk_Data);

	//Converting objects from Label Image into object types
	Classify_Data(rgb, LabelImageBW, LabelImageColour, nlabelBW, nlabelColour, Bulk_Data, Robot_Data, Opponent_Data, Obstacle_Data, Player);

	//Classify_Data_Troubleshooting(rgb, Robot_Data, Opponent_Data, Obstacle_Data);

	//Shoot laser if defender is in sight
	//Shoot_Laser (rgb, Robot_Data, Opponent_Data, Obstacle_Data, pw_laser, laser);

	//Detect if current motion will result in a collision and prevent collision
	Collision_Detection(rgb, Robot_Data, Opponent_Data, Obstacle_Data, pw_l, pw_r);

	//Hunt defender
	int Ic[4], Jc[4];
	vector<int> OL;
	copy(ProcessingImage, a);
	find_hollow_circles(nlabelBW, LabelImageBW, a, rgb, Ic, Jc);
	find_obstacles(rgb, LabelImageColour, a, nlabelColour, OL);
	detect_obstruction(Ic, Jc, OL, rgb, LabelImageColour,detected);
	if (detected == false) opponent_track(Ic, Jc, rgb, LabelImageBW, pw_r, pw_l,laser);
	else{
		int id, jd;
		find_path(Ic, Jc, OL, rgb, LabelImageColour, id, jd);
		go_to(Ic, Jc, pw_l, pw_r, rgb, id, jd);
		
	}

	//pause();

	free_image(LabelImageBW);
	free_image(LabelImageColour);
	free_image(ProcessingImage);
	free_image(a);
}

//Anthony's functions
void find_hollow_circles(int& nlabels, image& label, image&a, image& rgb0, int Ic[4], int Jc[4]) {
	ibyte* p, * pc;
	i2byte* pl, * plc;

	int R, G, B;

	pl = (i2byte*)label.pdata;

	int i, j,height;
	double ic, jc;
	int kc;

	int width = rgb0.width;
	p = rgb0.pdata;


	for (int label_num = 1; label_num <= nlabels; label_num++) {

		centroid(a, label, label_num, ic, jc);
		kc = (int)ic + (int)jc * width;

		plc = pl + kc;
		pc = p + 3 * kc;

		B = *pc;
		G = *(pc + 1);
		R = *(pc + 2);


		if (*plc == 0) {

			//green
			if (R < 120 && G >120 && B < 150) {
				//draw_point_rgb(rgb0, ic, jc, 255, 255, 255);
				Ic[0] = ic;
				Jc[0] = jc;
			}
			//Red
			else if (R > 100 && G < 100 && B < 100) {
				//draw_point_rgb(rgb0, ic, jc, 255, 255, 255);
				Ic[1] = ic;
				Jc[1] = jc;
			}
			//orange
			else if (R > 220 && G > 130 && G < 200 && B < 150) {
				//draw_point_rgb(rgb0, ic, jc, 255, 255, 255);
				Ic[2] = ic;
				Jc[2] = jc;
			}
			//blue
			else if (R < 60 && G < 175 && B > 200) {
				//draw_point_rgb(rgb0, ic, jc, 255, 255, 255);
				Ic[3] = ic;
				Jc[3] = jc;
			}
		}
	}

}

void clean_up(image& rgb, image&a) {
	int width, height;
	image b;

	width = rgb.width;
	height = rgb.height;

	b.type = GREY_IMAGE;
	b.width = width;
	b.height = height;

	allocate_image(b);

	copy(rgb, a);
	threshold(a, b, 70);
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

void find_obstacles(image& rgb, image& label, image& a, int nlabel, vector<int>& OL) {
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

		if (diff < 3 && diff>0 &&min>10) {//by trial and error below 3 were the obstacles for diff and min 10 is to remove noise
			OL.push_back(l);
			centroid(a, label, l, ic, jc);
			//draw_point_rgb(rgb, ic, jc, 255, 0, 0); //For Troubleshooting
		}
	}
}

void opponent_track(int Ic[4], int Jc[4], image& rgb, image& label, int& pw_r, int& pw_l, int &laser) {

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
			laser = 0;
		}
		else if (Jc[1] > (y + deadzone)) {
			pw_r = 1000;
			pw_l = 1000;
			laser = 0;
		}
		else {
			pw_r = 1500;
			pw_l = 1500;
			laser = 1;
		}
	}

	else if (id > Ic[0]) {
		if (Jc[1] < (y - deadzone)) {
			pw_r = 1000;
			pw_l = 1000;
			laser = 0;
		}
		else if (Jc[1] > (y + deadzone)) {
			pw_r = 2000;
			pw_l = 2000;
			laser = 0;
		}
		else {
			pw_r = 1500;
			pw_l = 1500;
			laser = 1;
		}
	}

	//drawing point
	//draw_point_rgb(rgb, id, jd, 255, 0, 255); //draw point of where the robot looks

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

	int X[2] = { min(cr_x,cd_x),max(cr_x,cd_x) };
	int Y[2] = { min(cr_y,cd_y),max(cr_y,cd_y) };

	if (dx == 0) {
		for (int k = 0; k < size; k++) {
			int i = k % width;
			int j = (k - i) / width;
			if (i > X[0] && i<X[1] && j>Y[0] && j < Y[1]) {
				if (abs(i - (int)cr_x) <= 1) {
					//draw_point_rgb(rgb, i, j, 255, 0, 0);
					for (int a : OL) {
						if (pl[k] == a) obstruction = true;
					}
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
			if (i > X[0] && i<X[1] && j>Y[0] && j < Y[1]) {
				if (abs(j - (int)y) <= 1) {
					//draw_point_rgb(rgb, i, j, 255, 0, 0);
					for (int a : OL) {
						if (pl[k] == a) obstruction = true;
					}
				}
			}

		}
	}
	detected = obstruction;
}

void go_to(int Ic[4], int Jc[4], int& pw_l, int& pw_r, image& rgb, int id, int jd) {
	int i, j, k, width, size, height;
	width = rgb.width;
	height = rgb.height;
	size = width * height;
	bool locked = false;

	//draw_point_rgb(rgb, id, jd, 255, 0, 0);

	float angle_of_robot = atan2(Jc[0] - Jc[1], Ic[0] - Ic[1]);
	float V[3]; //0-x, 1-y, 2-angle
	V[0] = (Ic[0] - id);
	V[1] = (Jc[0] - jd);
	V[2] = atan2(jd - Jc[0], id - Ic[0]);
	float angle_error = V[2] - angle_of_robot;
	if (angle_error < -0.1) {
		pw_l = 1000;
		pw_r = 1000;
	}
	else if (angle_error > 0.1) {
		pw_l = 2000;
		pw_r = 2000;
	}
	else {
		float tol = 10;
		if (fabs(V[1]) > tol || fabs(V[0]) > tol) {
			pw_l = 2000;
			pw_r = 1000;
		}
		else {
			pw_l = 1500;
			pw_r = 1500;
		}
	}
}
//this is a simplified patroling function
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

void find_path(int Ic[4], int Jc[4], vector<int>& OL, image& rgb, image& label, int& id, int& jd) {
	//draw to lines one vertical and horizontal and see if you mvoe there it is in the clear
	int width = rgb.width;
	int height = rgb.height;
	i2byte* pl = (i2byte*)label.pdata;

	bool invalidh = false;
	bool invalidv = false;

	int x1 = Ic[0];
	int y1 = Jc[0];
	int x2 = (Ic[2] + Ic[3]) / 2;
	int y2 = (Jc[2] + Jc[3]) / 2;
	int ix = x2;
	int jy = y2;

	int xs = min(x1, x2);
	int xe = max(x1, x2);
	int ys = min(y1, y2);
	int ye = max(y1, y2);

	// Increase the search space in steps
	for (int step = 0; step < 50; step++) {
		xs = xs - step;
		xe = xe + step;
		ys = ys - step;
		ye = ye + step;

		//boundaries
		if (xs < 0) xs = 30;
		if (xe >= width) xe = width - 30;
		if (ys < 0) ys = 30;
		if (ye >= height) ye = height - 30;

		//horizontal
		for (int i = xs; i <= xe; i++) {
			int k = i + jy * width;
			if (pl[k] != 0) {
				invalidh = true;
				break;
			}
		}

		//vertical
		for (int j = ys; j <= ye; j++) {
			int k = ix + j * width;
			if (pl[k] != 0) {
				invalidv = true;
				break;
			}
		}

		// horizontal clear, vertical blocked
		if (!invalidh && invalidv) {
			id = x2 + step;
			jd = y1;
			break;
		}
		// horizontal blocked, vertical clear
		else if (invalidh && !invalidv) {
			id = x1;
			jd = y2 + step;
			break;
		}
		// both clear
		else if (!invalidh && !invalidv) {
			id = x2 + step;
			jd = y1;
			break;
		}
	}
}

//Fred's functions

/*
int auto_select_shape_by_size(i2byte& nlabel, image& label)
// select an object from a binary image based on its area
// use instead of the select_object function in the
// find_object function
{
#define MAX_LABELS 256
	int labelAreas[MAX_LABELS] = { 0 };
	int wheelArea = 500;
	i2byte* pl;

	acquire_image_sim(rgb0);
	label_objects(tvalue);

	pl = (i2byte*)label.pdata;

	//measuring the area (amount of pixels) of a shape
	for (int y = 0; y < label.height; y++) {
		for (int x = 0; x < label.width; x++) {
			int labelVal = *(pl + y * label.width + x); //get label at pixel (x,y)
			if (labelVal > 0 && labelVal < MAX_LABELS) {
				labelAreas[labelVal]++; //increment that label's area count
			}
		}
	}

	//checks each shape that are of a certain area
	//if multiple, only selects one (the last one)
	for (int l = 1; l < MAX_LABELS; l++) {
		if (labelAreas[l] < 1000 && labelAreas[l]>100) {
			wheelArea = labelAreas[l];
			nlabel = l;
		}
	}
	return 0; // no errors
}
*/
//my functions also use the find_hollow_circles and clean_up functions of Anthony

/*
int find_obstacles(image& rgb, image& label, image& a, int obs_x[], int obs_y[], int max_obs) {
	const int MAX_LABELS = 255;
	int labelAreas[MAX_LABELS] = { 0 };
	int n_obstacles = 0;
	i2byte* pl;

	pl = (i2byte*)label.pdata;


	//measuring the area (amount of pixels) of a shape
	for (int y = 0; y < label.height; y++) {
		for (int x = 0; x < label.width; x++) {
			int labelVal = *(pl + y * label.width + x); //get label at pixel (x,y)
			if (labelVal > 0 && labelVal < MAX_LABELS) {
				labelAreas[labelVal]++; //increment that label's area count
			}
			
		}
	}
	

	for (int label_id = 1; label_id <= 255; label_id++) {
		double ic, ij;
		centroid(a, label, label_id, ic, ij);
		if (labelAreas[label_id] > 2800) {
			//cout << "\nlabel = " << label_id;
			if (n_obstacles < max_obs) {
				obs_x[n_obstacles] = (int)ic;
				obs_y[n_obstacles] = (int)ij;
				n_obstacles++;
			}
		}
	}
	//cout << "\n_obs = " << n_obstacles;
	return n_obstacles;
}
*/
double estimate_radius_from_image(image& label, double IC, double JC) {
	double radius, radius_max, radius_div, arc, arc_max, arc_div;
	int i, j, width, height, k;
	const double PI = 3.14159;
	ibyte* pc;
	ibyte r, g, b;

	pc = label.pdata;
	radius_max = 100;
	radius_div = 1;
	arc_div = 1;
	width = label.width;
	height = label.height;


	for (radius = 1; radius < radius_max; radius += radius_div) {
		arc_max = 2 * PI * radius;
		for (arc = 0; arc <= arc_max; arc += arc_div) {
			double beta = arc / radius;
			i = (int)(IC + radius * cos(beta));
			j = (int)(JC + radius * sin(beta));

			// limit (i,j) from going off the image
			if (i < 0) i = 0;
			if (i > width - 1) i = width - 1;
			if (j < 0) j = 0;
			if (j > height - 1) j = height - 1;

			k = i + width * j;
			r = pc[3 * k + 0];
			g = pc[3 * k + 1];
			b = pc[3 * k + 2];

			if (r > 250 && g > 250 && b > 250) {
				return radius; // obstacle ends, return radius
			}
			
		}
	}
	return radius_max; // max radius if full disk
}
/*
bool is_robot_in_line_of_sight(int defender_x, int defender_y, int opponent_x, int opponent_y, image& rgb, image& label, image& a) {
	const int MAX_OBS = 100;
	int obs_x[MAX_OBS], obs_y[MAX_OBS];
	int num_obs = find_obstacles(rgb, label, a, obs_x, obs_y, MAX_OBS);

	double dx, dy, fx, fy, a_, b_, c, sqrt_quad, t1, t2;


	dx = defender_x - opponent_x;
	dy = defender_y - opponent_y;
	


	//check if the opponent line of sight is obstructed by an obstacle
	for (int i = 0; i < num_obs; i++) {
		fx = obs_x[i] - opponent_x;
		fy = obs_y[i] - opponent_y;

		double radius = estimate_radius_from_image(rgb, obs_x[i], obs_y[i]);

		a_ = dx * dx + dy * dy;
		b_ = 2 * (fx * dx + fy * dy);
		c = fx * fx + fy * fy - radius * radius;

		sqrt_quad = b_ * b_ - 4 * a_ * c;

		if (sqrt_quad >= 0) {
			
			sqrt_quad = sqrt(sqrt_quad);
			t1 = (-b_ - sqrt_quad) / (2 * a_);
			t2 = (-b_ + sqrt_quad) / (2 * a_);
			// If the line from robot to opponent intersects this obstacle
			if ((t1 >= 0 && t1<= 1) || (t2 >= 0 && t2 <= 1)) {
				return false;  // Line of sight is blocked
			}
			
		}

	}
	return true;
}
*/
/*
void find_hiding_position(int defender_x, int defender_y, int opponent_x, int opponent_y, image& rgb, image& label, image& a, double& hide_x, double& hide_y) {
	const int MAX_OBS = 100;
	int obs_x[MAX_OBS], obs_y[MAX_OBS];
	int num_obs = find_obstacles(rgb, label, a, obs_x, obs_y, MAX_OBS);

	const double PI = 3.14159265358979323846;

	// Otherwise, find a better hiding spot
	double best_score = 1e6; // <- lower score is better (NOT 1e9!)

	for (int i = 0; i < num_obs; i++) {
		// Direction from opponent to obstacle
		double dx = obs_x[i] - opponent_x;
		double dy = obs_y[i] - opponent_y;
		double dist_to_opp_sq = dx * dx + dy * dy;

		// Candidate hiding position (behind obstacle relative to opponent)
		double candidate_x = obs_x[i] + dx * 0.5;
		double candidate_y = obs_y[i] + dy * 0.5;

		// Distance from defender to this hiding spot
		double dist_to_robot_sq = (candidate_x-defender_x)*(candidate_x-defender_x)+(candidate_y-defender_y)*(candidate_y-defender_y);

		// Scoring function: prefer obstacles far from opponent, but not too far from us
		double score = dist_to_opp_sq - 0.5 * dist_to_robot_sq;

		if (score < best_score) {
			best_score = score;
			hide_x = candidate_x;
			hide_y = candidate_y;
		}
	}

}
*/
/*
void navigate_to_target(robot* defender, double hide_x, double hide_y, image& rgb, image& label, image& a, int& pw_l, int& pw_r) {
	const int MAX_OBS = 100;
	int obs_x[MAX_OBS], obs_y[MAX_OBS];
	int num_obs = find_obstacles(rgb, label, a, obs_x, obs_y, MAX_OBS);
	
	double rx = defender->x[2];
	double ry = defender->x[3];
	double rtheta = defender->x[1];
	
	const double PI = 3.14159265358979323846;
	
	double dx = hide_x - rx;
	double dy = hide_y - ry;
	
	double dist = sqrt(dx * dx + dy * dy);
	
	// Obstacle repulsion
	double repulse_x = 0;
	double repulse_y = 0;
	for (int i = 0; i < num_obs; i++) {
		double obs_dx = rx - obs_x[i];
		double obs_dy = ry - obs_y[i];
		double dist_sq = obs_dx * obs_dx + obs_dy * obs_dy;
		if (dist_sq < 10000 && dist_sq > 1.0) {
			double force = 10000.0 / dist_sq;
			repulse_x += force * obs_dx;
			repulse_y += force * obs_dy;
		}
	}
	
	dx += repulse_x;
	dy += repulse_y;
	
	
	double angle_to_target = atan2(dy, dx);
	double angle_diff = angle_to_target - rtheta;

	while (angle_diff > PI) angle_diff -= 2 * PI;
	while (angle_diff < -PI) angle_diff += 2 * PI;

	
	//if more than 80 pixels away, forward_speed = 0.5
	//if less, forward_speed = 0.2
	double forward_speed = (dist > 80) ? 0.5 : 0.2;
	double turn_speed = 0.3;

	
	//if not facing target within 0.1rad (6deg), turn
	if (angle_diff > 0.1) {
		defender->set_inputs(1500 - forward_speed * 400, 1500 + turn_speed * 400, 1500, 0);
	}
	else if (angle_diff < -0.1) {
		defender->set_inputs(1500 + turn_speed * 400, 1500 - forward_speed * 400, 1500, 0);
	}
	else {
		defender->set_inputs(1500 + forward_speed * 400, 1500 + forward_speed * 400, 1500, 0);
	}

	const int pw_base = 1500;
	const int max_delta = 500;
	
	int speed = (int)(dist * 5);
	if (speed > 300) speed = 300;
	if (dist < 5) speed = 0;

	int turn = (int)(angle_diff * 400);
	if (turn > max_delta) turn = max_delta;
	if (turn < -max_delta) turn = -max_delta;
	if (dist < 30 && fabs(turn) > 100) {
		turn /= 2;
	}


	pw_l = pw_base - speed + turn;
	pw_r = pw_base + speed + turn;

	if (pw_l < 1000) pw_l = 1000;
	if (pw_l > 2000) pw_l = 2000;
	if (pw_r < 1000) pw_r = 1000;
	if (pw_r > 2000) pw_r = 2000;
}
*/
/*
void dynamic_hide(int defender, image& rgb, image& rgb0, image& label, image& a, image& b, int& pw_l, int& pw_r) {
	int Ic[4], Jc[4], nlabel;
	static double hide_x = 0;
	static double hide_y = 0;

	static double last_opp_x = 0;
	static double last_opp_y = 0;

	copy(rgb, rgb0);
	clean_up(rgb, a);
	label_image(a, label, nlabel);
	find_hollow_circles(nlabel, rgb, label, a, rgb0, Ic, Jc);

	int defender_x = (Ic[0] + Ic[1]) / 2;
	int defender_y = (Jc[0] + Jc[1]) / 2;
	int opponent_x = (Ic[2] + Ic[3]) / 2;
	int opponent_y = (Jc[2] + Jc[3]) / 2;



	bool exposed = is_robot_in_line_of_sight(defender_x, defender_y, opponent_x, opponent_y, rgb0, label, a);

	double dx = opponent_x - last_opp_x;
	double dy = opponent_y - last_opp_y;
	double opponent_speed = sqrt(dx * dx + dy * dy);

	bool opponent_moved = opponent_speed > 1.0; // Movement threshold
	

	if (exposed) {
		find_hiding_position(defender_x, defender_y, opponent_x, opponent_y, rgb0, label, a, hide_x, hide_y);
	}
	navigate_to_target(defender, hide_x, hide_y, rgb0, label, a, pw_l, pw_r);
	
	last_opp_x = opponent_x;
	last_opp_y = opponent_y;
}
*/

//Marc's functions
void Collision_Detection(image& rgb, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data, int& pw_l, int& pw_r) {
    const int width = rgb.width;
    const int height = rgb.height;
	bool Obstruction;
	
    // Define a scan zone in front of the robot
    const int safe_distance_px = 30;
    const int scan_width = 5;
	
	// Get robot's current position and heading
    double theta = Robot_Data[0][2];
    double rx = (Robot_Data[0][0]+ Robot_Data[1][0])/2;
	double ry = (Robot_Data[0][1] + Robot_Data[0][1]) / 2;


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

            // check for objects
            Obstruction = ObstacleAtLocation(i,j,Obstacle_Data);
            if (Obstruction) {
                // Something is directly ahead!
               // cout << "\n[Vision Collision] Label detected at (" << i << ", " << j << ")";

        		int delta_l = abs(pw_l - 1500);
				int delta_r = abs(pw_r - 1500);

				if (((delta_l > delta_r) && (pw_l > pw_r)) || ((delta_l < delta_r) && (pw_l < pw_r))){
                    // turning left -> turn harder left
                    pw_l = 2000; // backwards left
                    pw_r = 2000; // forward right
				} else if (((delta_l > delta_r) && (pw_l < pw_r)) || ((delta_l < delta_r) && (pw_l > pw_r))){
                    // turning right -> turn harder right
                    pw_l = 1000; // forward left
                    pw_r = 1000; // backwards right
				} else if( (delta_l == delta_r) && (pw_l != pw_r) && ( pw_l < 1500 && pw_r > 1500) ){
				// if velocity was forward -> turns backwards
				pw_l = 2000; // backwards left
				pw_r = 1000; // backwards right 
				}
				return;
            }
        }
    }
}


//Jacob Functions

//Determine if there is an obstacle at a given location
static bool ObstacleAtLocation(int I, int J, vector<array<int, 6>>& Obstacle_Data) {
	int k;

	//Check to see if any Obstacle is at location (I, J)
	for (k = 0; k < Obstacle_Data.size(); k++) {
		if (Obstacle_Data[k][0] == I && Obstacle_Data[k][1] == J) return 1; //If obstacle at (I, J) return true
	}

	//If no obstacle at (I, J), return false
	return 0;
}
//Creates a greyscale image. Threshold set to identify black/dark items
static void BWProcessing(image& InputImage, image& OutputImage) {
	int width, height, Threshold;
	image TempImageA, TempImageB, View;

	width = InputImage.width;
	height = InputImage.height;

	TempImageA.type = GREY_IMAGE;
	TempImageA.width = width;
	TempImageA.height = height;

	TempImageB.type = GREY_IMAGE;
	TempImageB.width = width;
	TempImageB.height = height;

	View.type = RGB_IMAGE;
	View.width = width;
	View.height = height;

	Threshold = 75;

	allocate_image(TempImageA);
	allocate_image(TempImageB);
	allocate_image(View);

	//Convert into greyscale, and scale brightness to cover full range.
	copy(InputImage, TempImageA);
	scale(TempImageA, TempImageB);
	lowpass_filter(TempImageB, TempImageA);

	/*
	//Make a Histogram
	int nhist, j;
	double hist[255], hmin, hmax, x;

	nhist = 60;
	histogram(TempImageA, hist, nhist, hmin, hmax);
	ofstream fout("BW_Hist.csv");
	for (j = 0; j < nhist; j++) {
		x = hmin + (hmax - hmin) / nhist * j;
		fout << x << "," << hist[j] << "\n";
	}
	fout.close();
	*/

	//Perform opperations to reduce noise
	threshold(TempImageA, TempImageB, Threshold);
	invert(TempImageB, TempImageA);
	erode(TempImageA, TempImageB);
	erode(TempImageB, TempImageA);
	dialate(TempImageA, TempImageB);
	dialate(TempImageB, TempImageA);

	//For troubleshooting
	/*
	cout << "\nImage processing complete. BW objects found";
	copy(TempImageA, View);
	save_rgb_image("BWProcessedImage.bmp", View);
	pause();
	*/

	copy(TempImageA, OutputImage);

	free_image(TempImageA);
	free_image(TempImageB);
	free_image(View);
}

//Creates a greyscale image. Threshold set to identify colourful/bright items
static void ColourProcessing(image& InputImage, image& OutputImage) {
	int width, height, Threshold;
	image TempImageA, TempImageB, View;

	width = InputImage.width;
	height = InputImage.height;

	TempImageA.type = GREY_IMAGE;
	TempImageA.width = width;
	TempImageA.height = height;

	TempImageB.type = GREY_IMAGE;
	TempImageB.width = width;
	TempImageB.height = height;

	View.type = RGB_IMAGE;
	View.width = width;
	View.height = height;

	Threshold = 237;

	allocate_image(TempImageA);
	allocate_image(TempImageB);
	allocate_image(View);

	//Convert into greyscale, and scale brightness to cover full range.
	copy(InputImage, TempImageA);
	scale(TempImageA, TempImageB);
	lowpass_filter(TempImageB, TempImageA);

	//For troubleshooting
	/*
	//Make a Histogram
	int nhist, j;
	double hist[255], hmin, hmax, x;

	nhist = 60;
	histogram(TempImageA, hist, nhist, hmin, hmax);
	ofstream fout("Colour_Hist.csv");
	for (j = 0; j < nhist; j++) {
		x = hmin + (hmax - hmin) / nhist * j;
		fout << x << "," << hist[j] << "\n";
	}
	fout.close();
	cout << "\nHistogram created. Greyscale image";
	copy(TempImageA, View);
	save_rgb_image("ColourGreyscaleImage.bmp", View);
	pause();
	*/


	//Perform opperations to reduce noise
	threshold(TempImageA, TempImageB, Threshold);
	invert(TempImageB, TempImageA);
	erode(TempImageA, TempImageB);
	erode(TempImageB, TempImageA);
	dialate(TempImageA, TempImageB);
	dialate(TempImageB, TempImageA);

	//For troubleshooting
	/*
	cout << "\nImage processing complete. Colour objects Found";
	copy(TempImageA, View);
	save_rgb_image("ColourProcessedImage.bmp", View);
	pause();
	*/

	copy(TempImageA, OutputImage);

	free_image(TempImageA);
	free_image(TempImageB);
	free_image(View);
}

//Labels the black and colourful greyscale images
static void Process_Image(image& InputImage, image& LabelImageBW, image& LabelImageColour, int& nlabelBW, int& nlabelColour) {

	image TempImage;

	TempImage.type = GREY_IMAGE;
	TempImage.width = InputImage.width;
	TempImage.height = InputImage.height;

	allocate_image(TempImage);

	BWProcessing(InputImage, TempImage);
	label_image(TempImage, LabelImageBW, nlabelBW);

	ColourProcessing(InputImage, TempImage);
	label_image(TempImage, LabelImageColour, nlabelColour);

	free_image(TempImage);
}

//Get data in the format: [Ic, Jc, theta, R, G, B] for all labeled objects from the black and colourful image processing
static void Get_Image_Data(image& rgb, image& LabelImageBW, image& LabelImageColour, int nlabelBW, int nlabelColour, vector<array<int, 6>>& Bulk_Data) {

	//Create Local Variables
	int i, Pixel_Number, k;
	int height, width;
	double C[2];
	ibyte* p;
	image TempImageA, TempImageB;

	//Initialize Variables
	height = rgb.height;
	width = rgb.width;
	p = rgb.pdata;
	C[0] = 0, C[1] = 0;

	TempImageA.type = GREY_IMAGE;
	TempImageA.width = rgb.width;
	TempImageA.height = rgb.height;

	TempImageB.type = GREY_IMAGE;
	TempImageB.width = rgb.width;
	TempImageB.height = rgb.height;

	//Create a local greyscale image for centroid function
	allocate_image(TempImageA);
	allocate_image(TempImageB);
	copy(rgb, TempImageA);
	scale(TempImageA, TempImageB);
	lowpass_filter(TempImageB, TempImageA);

	
	Bulk_Data.resize(nlabelBW + nlabelColour); //Updating size of Bulk_Data to prevent accessing out of range of the matrix

	//Get Position Data for BW Objects
	for (k = 1; k <= nlabelBW; k++) {
		centroid(TempImageA, LabelImageBW, k, C[0], C[1]);
		for (i = 0; i < 2; i++) {
			Bulk_Data[k - 1][i] = static_cast<int>(round(C[i]));
		}
	}
	//Get Position Data for Colourful Objects
	for (k = 1; k <= nlabelColour; k++) {
		centroid(TempImageA, LabelImageColour, k, C[0], C[1]);
		for (i = 0; i < 2; i++) {
			Bulk_Data[k - 1 + nlabelBW][i] = static_cast<int>(round(C[i]));
		}
	}
	//Get Colour Data for All Objects at their centroid
	for (k = 0; k < (nlabelBW + nlabelColour); k++) {
		Pixel_Number = (Bulk_Data[k][1] * width) + Bulk_Data[k][0];
		Bulk_Data[k][3] = p[3 * Pixel_Number + 2];  //R
		Bulk_Data[k][4] = p[3 * Pixel_Number + 1];  //G
		Bulk_Data[k][5] = p[3 * Pixel_Number];		//B
	}

	//For TroubleShooting
	/*
	cout << "\nHere is the object data:\n";
	for (k = 0; k < (nlabelBW + nlabelColour); k++) {
		cout << "\nObject " << (k) << " Data: [";
		for (i = 0; i < 5; i++) {
			cout << Bulk_Data[k][i] << ", ";
		}
		cout << Bulk_Data[k][5] << "]";
		draw_point_rgb(rgb, Bulk_Data[k][0], Bulk_Data[k][1], 255, 255, 255);
	}
	*/

	//Release image memory
	free_image(TempImageA);
	free_image(TempImageB);
}

//Modified version of Anthony's find_hollow_circles function
static bool Hollow_Circle(image& rgb, image& LabelImage, int& nlabels,  int Ic, int Jc) {
	
	//Initialize Variables
	i2byte* pl, * plc;
	int width, height, Pixel_Number;

	//Define Local Variables
	pl = (i2byte*)LabelImage.pdata; //Pointer to label image
	width = rgb.width;
	height = rgb.height;

	//Function
	Pixel_Number = (Jc*width) + Ic; //Pixel number of the centroid (greyscale image) 
	plc = pl + Pixel_Number; //Pixel number of the centroid (label image)

	//Logic: If label at centroid is 0, then the centroid is on the background. Therefore, the object is a hollow circle
	if (*plc == 0) {
		return(1);
	}
	else {
		return(0);
	}
}

static bool NotGrey(int R, int G, int B) {
	double MaxValue, MinValue, Range, Average;

	MaxValue = R;
	if (G > MaxValue)  MaxValue = G;
	if (B > MaxValue)  MaxValue = B;

	MinValue = R;
	if (G < MinValue)  MinValue = G;
	if (B < MinValue)  MinValue = B;

	Range = MaxValue - MinValue;
	Average = (R + G + B) / 3.0;

	if (Range < 20 && Average > 127.5) {//If all values of RGB are similar and the object is bright, it is grey
		return 0;
	}
		
	else {
		return 1;
	}
}
//Determine colour of object based on possible colour types
static ColourTypes Object_Colour (int R, int G, int B) {


	if (R < 120 && G > 120 && B < 150)
		return GREEN;
	else if (R > 100 && G < 100 && B < 100)
		return RED;
	else if (R > 220 && G > 130 && G < 200 && B < 150)
		return ORANGE;
	else if (R < 60 && G < 175 && B > 200)
		return BLUE;
	else
		return WRONG;
}

static double Calculate_Theta(double x1, double y1, double x2, double y2) {
	double dx, dy, angle;

	dx = x2 - x1;
	dy = y2 - y1;
	angle = atan2(dy, dx);
	return angle;
}

//Assign Bulk Data to either Robot or Opponent
//Using Array notation to be able to directly copy the rows of the matrices
static void Assign_Robot_Data(char Player, ColourTypes Colour,array<int, 6>&Bulk_Data, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data) {
	if (Player == 'A') {
		switch (Colour) {
		case GREEN:  Robot_Data[0] = Bulk_Data; break; //Row 1 of Robot_Data is equal to the current row of Bulk_Data
		case RED:    Robot_Data[1] = Bulk_Data; break; //Row 2 of Robot_Data is equal to the current row of Bulk_Data
		case ORANGE: Opponent_Data[0] = Bulk_Data; break;
		case BLUE:   Opponent_Data[1] = Bulk_Data; break;
		default: break;
		}
	}
	else if (Player == 'B') {
		switch (Colour) {
		case GREEN:  Opponent_Data[0] = Bulk_Data; break;
		case RED:    Opponent_Data[1] = Bulk_Data; break;
		case ORANGE: Robot_Data[0] = Bulk_Data; break;
		case BLUE:   Robot_Data[1] = Bulk_Data; break;
		default: break;
		}
	}
}

//Separate the Bulk Data into 3 object types: our robot, opponent robot, and obstacles.
static void Classify_Data(image&rgb, image& LabelImageBW, image& LabelImageColour, int nlabelBW, int nlabelColour, vector<array<int, 6>>& Bulk_Data, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int,6>>& Obstacle_Data, char Player) {

	//Initialize Local Variables
	int k, i, j, accumulator;
	image TempImageA, TempImageB;
	ColourTypes Colour;

	//Define Local Variables
	accumulator = 0;
	TempImageA.type = GREY_IMAGE;
	TempImageA.width = rgb.width;
	TempImageA.height = rgb.height;

	TempImageB.type = GREY_IMAGE;
	TempImageB.width = rgb.width;
	TempImageB.height = rgb.height;

	//Create local greyscale image for find_hollow_circles function
	allocate_image(TempImageA);
	allocate_image(TempImageB);
	copy(rgb, TempImageA);
	scale(TempImageA, TempImageB);
	lowpass_filter(TempImageB, TempImageA);

	//Identify Player and Opponent
	//Uses black/dark processed image for better accuracy.
	//The black/dark processed image has less noise than the colourful/bright processed image
	for (k = 0; k < nlabelBW; k++) {
		if (Hollow_Circle(rgb, LabelImageBW, nlabelBW, Bulk_Data[k][0], Bulk_Data[k][1])) {
			Colour = Object_Colour(Bulk_Data[k][3], Bulk_Data[k][4], Bulk_Data[k][5]);

			if (Colour == WRONG){
				cout << "\nTHere was an error in determining the colour of the robots.";
				break;
				}
			Assign_Robot_Data(Player, Colour, Bulk_Data[k], Robot_Data, Opponent_Data);
		}
	}

	//Add Angle data to Robot and Opponent
	//The first row of opponent and robot is always the "head" of the robot
	for (i = 0; i < 2; i++) {
		Robot_Data[i][2] = Calculate_Theta(Robot_Data[0][0], Robot_Data[0][1], Robot_Data[1][0], Robot_Data[1][1]);
		Opponent_Data[i][2]= Calculate_Theta(Opponent_Data[0][0], Opponent_Data[0][1], Opponent_Data[1][0], Opponent_Data[1][1]);
	}

	//Identify Obstacles
	vector<int> Obstacle_Objects; //1D dynamic array containing the object number of the obstacles
	find_obstacles(rgb, LabelImageColour, TempImageA, nlabelColour, Obstacle_Objects);
	
	for (i = 0; i < Obstacle_Objects.size(); i++) {
		Obstacle_Objects[i] += nlabelBW; //Increment each label number by the number of BW labels since the BW labels come before the Colour labels in Bulk_Data
	}

	Obstacle_Data.resize(Obstacle_Objects.size()); //Size obstacle_data to prevent out-of-range errors

	//Of the objects identified as obstacles, only the objects which are "NotGrey" are actually obstacles
	for (i = 0; i < Obstacle_Objects.size(); i++) {
		j = Obstacle_Objects[i] - 1;
		if (NotGrey(Bulk_Data[j][3], Bulk_Data[j][4], Bulk_Data[j][5])) {
			Obstacle_Data[accumulator] = Bulk_Data[j]; //Only update when object isn't grey
			accumulator++;
		}
	}
	Obstacle_Data.resize(accumulator); //Truncate the trailing rows of 0s
	
	//Add obstacle size data to the Obstacle_Data
	//for (i = 0; i < Obstacle_Data.size(); i++) {
	//	Obstacle_Data[i][2] = static_cast <int>(ceil(estimate_radius_from_image(LabelImageColour, Obstacle_Data[i][0], Obstacle_Data[i][1]))); //Want the round-up integer size of the obstacle
	//}

	//Release image memory
	free_image(TempImageA);
	free_image(TempImageB);
}

static void Classify_Data_Troubleshooting(image& rgb, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data) {
	int i, j;
	image TempImage;

	TempImage.type = RGB_IMAGE;
	TempImage.width = rgb.width;
	TempImage.height = rgb.height;

	allocate_image(TempImage);
	copy(rgb, TempImage);

	//Draw black Point on robot
	for (i = 0; i < Robot_Data.size(); i++) {
		draw_point_rgb(TempImage, Robot_Data[i][0], Robot_Data[i][1], 0, 0, 0);
	}
	//Draw Grey Point on opponent
	for (i = 0; i < Opponent_Data.size(); i++) {
		draw_point_rgb(TempImage, Opponent_Data[i][0], Opponent_Data[i][1], 147, 147, 147);
	}
	//Draw white point on obstacles
	for (i = 0; i < Obstacle_Data.size(); i++) {
		cout << "\nObject " << (i) << " Data: [";
		for (j = 0; j < 5; j++) {
			cout << Obstacle_Data[i][j] << ", ";
		}
		cout << Obstacle_Data[i][5] << "]";
	
		draw_point_rgb(TempImage, Obstacle_Data[i][0], Obstacle_Data[i][1], 255, 255, 255);
	}

	save_rgb_image("Classify_Data_Troubleshooting.bmp", TempImage);
	free_image(TempImage);
}

/*
static void Shoot_Laser(image& rgb, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data, int& pw_laser, int& laser) {
	int I_laser, J_laser;


}
*/