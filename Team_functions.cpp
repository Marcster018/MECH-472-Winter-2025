//mech 472 functions

#include "Team_functions.h"
#include <vector>
#include <array>

using namespace std;

//Anthony functions
void find_hollow_circles(int& nlabels, image& rgb, image& label, image&a, image& rgb0, int Ic[4], int Jc[4]) {
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

void find_obstacles(image& rgb, image& label, image& a, int nlabel, int ObsLabel[2]) {
	int i, j, k, l, height, width, size;
	double ic, jc;
	i2byte* pl;

	pl = (i2byte*)label.pdata;
	height = rgb.height;
	width = rgb.width;
	size = width * height;

	int dx[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
	int dy[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };
	
	float dmin1 = 1e9, dmin2 = 1e9;
	int label1 = 0, label2 = 0;

	for (l = 1; l <= nlabel; l++) {
		centroid(a, label, l, ic, jc);
		float R[8] = { 0 };

		for (int d = 0; d < 8; d++) {
			i = ic;
			j = jc;
			while (true) {
				i += dx[d];
				j += dy[d];
				if (i < 0 || j < 0 || i >= width || j >= height) break;

				k = i + j * width;
				if (pl[k] != l) break;

				float x = i - ic;
				float y = j - jc;
				R[d] = sqrt(x * x + y * y);
			}
		}

		float min = R[0], max = R[0];
		for (int d = 1; d < 8; d++) {
			if (R[d] < min) min = R[d];
			if (R[d] > max) max = R[d];
		}

		float diff = max - min;

		if (diff > 0) {
			if (diff < dmin1) {
				dmin2 = dmin1;
				label2 = label1;
				dmin1 = diff;
				label1 = l;
			}
			else if (diff < dmin2) {
				dmin2 = diff;
				label2 = l;
			}
		}
	}

	if (label1 != -1) {
		ObsLabel[0] = label1;
		centroid(a, label, label1, ic, jc);
		//draw_point_rgb(rgb, (int)ic, (int)jc, 255, 0, 0);
	}
	if (label2 != -1) {
		ObsLabel[1] = label2;
		centroid(a, label, label2, ic, jc);
		//draw_point_rgb(rgb, (int)ic, (int)jc, 255, 0, 0);
	}
}

void opponent_track(int Ic[4], int Jc[4], image& rgb,image &label, int& pw_r, int& pw_l) {

	int id = (Ic[3] + Ic[2]) / 2;
	int jd = (Jc[3] + Jc[2]) / 2;

	int width = rgb.width;
	int height = rgb.height;
	int size = width * height;

	int k,i,j;
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
	//cout << "\ny= " << m << " + " << b;

	//want to check if the red dot Ic[1], Jc[1] is bellow or above the line
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
	draw_point_rgb(rgb, id, jd, 255, 0, 255);

}

void detect_obstruction(int Ic[4], int Jc[4], int ObsLabel[2], image& rgb, image& label, bool &detected) {
	int width = rgb.width;
	int height = rgb.height;
	int size = width * height;
	bool obstruction=false;

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
				if (pl[k] == ObsLabel[0] || pl[k] == ObsLabel[1]) obstruction = true;
			}
		}
	}
	else {
		float m = dy / dx;
		float b = cr_y - cr_x * m;
		//line drawing

		for (int k = 0; k < size; k++) {
			int i = k % width;
			int j = (k - i) / width;
			float y = i * m + b;
			if (abs(j - (int)y) <= 1) {
				draw_point_rgb(rgb, i, j, 255, 0, 0);
				if (pl[k] == ObsLabel[0] || pl[k] == ObsLabel[1]) obstruction = true;
			}
		}
	}
	detected = obstruction;
}

void go_to(int Ic[4], int Jc[4], int& pw_l, int& pw_r, image& rgb,image &label, int id,int jd) {
	int i, j, k, width, size, height;
	width = rgb.width;
	height = rgb.height;
	size = width * height;
	bool locked = false;
	i2byte* pl;
	pl = (i2byte*)label.pdata;
	k = id + jd * width;

	if (pl[k] == 0) {
		int ir = (Ic[0] + Ic[1]) / 2;
		int jr = (Jc[0] + Jc[1]) / 2;

		int V[2] = { id - ir,jd - jr }; //V[0]=x, V[1]=y
		float angle = atan2(V[1], V[0]);
		float robot_angle = atan2(Jc[0] - Jc[1], Ic[0] - Ic[1]);
		float angle_error = angle - robot_angle;

		draw_point_rgb(rgb, id, jd, 200, 0, 0);

		if (fabs(angle_error) > 0.05) {
			if (angle_error > 0) {
				pw_l = 2000;
				pw_r = 2000;
			}
			else {
				pw_l = 1000;
				pw_r = 1000;
			}
			locked = false;
		}
		else {
			pw_l = 1500;
			pw_r = 1500;
			locked = true;
		}

		int x_error = abs(id - Ic[0]);
		int y_error = abs(jd - Jc[0]);

		if (x_error > 10 && y_error > 10) {
			if (locked == true) {
				pw_l = 1000;
				pw_r = 2000;
			}
		}
		else if (x_error > 2 && y_error > 2) {
			if (locked == true) {
				pw_l = 1250;
				pw_r = 1750;
			}
		}
		else if (x_error > 1 && y_error < 1){
			pw_l = 1500;
			pw_r = 1500;
		}
	}

}



//Fred functions
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

double estimate_radius_from_image(image& rgb_obstacle, double IC, double JC) {
	double radius, radius_max, radius_div, arc, arc_max, arc_div;
	int i, j, width, height, k;
	const double PI = 3.14159;
	ibyte* pc;
	ibyte r, g, b;

	pc = rgb_obstacle.pdata;
	radius_max = 100;
	radius_div = 1;
	arc_div = 1;
	width = rgb_obstacle.width;
	height = rgb_obstacle.height;


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

bool is_robot_in_line_of_sight(robot* defender, robot_system* S1, double x_obs[], double y_obs[], int N_obs) {
	double defender_x = defender->x[2];
	double defender_y = defender->x[3];
	double opponent_x = S1->P[2]->x[2];
	double opponent_y = S1->P[2]->x[3];
	double dx, dy, fx, fy, obs_x, obs_y, a, b, c, sqrt_quad, t1, t2;
	double angle_to_robot, opponent_angle, relative_angle;
	const double PI = 3.14159265358979323846;

	dx = defender_x - opponent_x;
	dy = defender_y - opponent_y;
	
	// Compute relative angle between defender direction and opponent
	angle_to_robot = atan2(dy, dx);
	opponent_angle = defender->x[1];
	//while (robot_angle > PI) robot_angle -= 2 * PI;
	//while (robot_angle < -PI) robot_angle += 2 * PI;
	relative_angle = angle_to_robot - opponent_angle;

	// Normalize the angle to be between -pi and pi
	while (relative_angle > PI) relative_angle -= 2 * PI;
	while (relative_angle < -PI) relative_angle += 2 * PI;

	//check if the opponent line of sight is obstructed by an obstacle
	for (int i = 0; i < N_obs; i++) {
		obs_x = x_obs[i];
		obs_y = y_obs[i];
		fx = obs_x - opponent_x;
		fy = obs_y - opponent_y;

		double radius = estimate_radius_from_image(rgb_obstacle[i], x_obs[i], y_obs[i]);

		a = dx * dx + dy * dy;
		b = 2 * (fx * dx + fy * dy);
		c = fx * fx + fy * fy - radius * radius;

		sqrt_quad = b * b - 4 * a * c;

		if (sqrt_quad >= 0) {
			
			sqrt_quad = sqrt(sqrt_quad);
			t1 = (-b - sqrt_quad) / (2 * a);
			t2 = (-b + sqrt_quad) / (2 * a);
			//cout << "\nt1 = " << t1 <<"  t2 = "<< t2;
			// If the line from robot to opponent intersects this obstacle
			if ((t1 >= 0 && t1<= 1) || (t2 >= 0 && t2 <= 1)) {
				return false;  // Line of sight is blocked
			}
			
		}

	}
	//cout << "\n angle = " << relative_angle;
	// If no obstacle blocks the view, check if robot is in opponent's field of view
	if (fabs(relative_angle) < (PI / 2)) { 
		return true; // The robot is in the opponent's line of sight
	}
	
	return false; // The robot is not in the opponent's line of sight
}

void find_hiding_position(robot_system* S1, double& hide_x, double& hide_y, double& hide_theta, double x_obs[], double y_obs[], int N_obs) {
	double rx = S1->P[1]->x[2]; // defender robot's x
	double ry = S1->P[1]->x[3]; // defender robot's y
	double opp_x = S1->P[2]->x[2]; // opponent x
	double opp_y = S1->P[2]->x[3]; // opponent y
	double opp_theta = S1->P[2]->x[1]; // opponent heading

	const double PI = 3.14159265358979323846;

	// Check if we are already in opponent's sight
	// If not seen, prefer to stay at current spot
	if (!is_robot_in_line_of_sight(S1->P[1], S1, x_obs, y_obs, N_obs)) {
		// No need to find a new hiding spot
		return;
	}

	// Otherwise, find a better hiding spot
	double best_score = 1e6; // <- lower score is better (NOT 1e9!)

	for (int i = 0; i < N_obs; i++) {
		double obs_x = x_obs[i];
		double obs_y = y_obs[i];

		// Direction from opponent to obstacle
		double dx = obs_x - opp_x;
		double dy = obs_y - opp_y;
		double dist_to_opp_sq = dx * dx + dy * dy;

		// Candidate hiding position (behind obstacle relative to opponent)
		double candidate_x = obs_x + dx * 0.5;
		double candidate_y = obs_y + dy * 0.5;

		// Distance from defender to this hiding spot
		double dx_robot = candidate_x - rx;
		double dy_robot = candidate_y - ry;
		double dist_to_robot_sq = dx_robot * dx_robot + dy_robot * dy_robot;

		// Scoring function: prefer obstacles far from opponent, but not too far from us
		double score = dist_to_opp_sq - 0.5 * dist_to_robot_sq;

		if (score < best_score) {
			best_score = score;
			hide_x = candidate_x;
			hide_y = candidate_y;
		}
	}

}

void navigate_to_target(robot* defender, double hide_x, double hide_y, double x_obs[], double y_obs[], int N_obs, int& pw_l, int& pw_r) {
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
	for (int i = 0; i < N_obs; i++) {
		double obs_dx = rx - x_obs[i];
		double obs_dy = ry - y_obs[i];
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

void dynamic_hide(robot* defender, robot_system* S1, double x_obs[], double y_obs[], int N_obs, int& pw_l, int& pw_r) {
	static double hide_x = 0;
	static double hide_y = 0;
	static double hide_theta = 0;
	static double last_opp_x = 0;
	static double last_opp_y = 0;

	double curr_opp_x = S1->P[2]->x[2];
	double curr_opp_y = S1->P[2]->x[3];

	bool exposed = is_robot_in_line_of_sight(defender, S1, x_obs, y_obs, N_obs);

	double dx = curr_opp_x - last_opp_x;
	double dy = curr_opp_y - last_opp_y;
	double opponent_speed = sqrt(dx * dx + dy * dy);

	bool opponent_moved = opponent_speed > 1.0; // Movement threshold
	find_hiding_position(S1, hide_x, hide_y, hide_theta, x_obs, y_obs, N_obs);

	if (exposed) {
		//find_hiding_position(S1, hide_x, hide_y, hide_theta, x_obs, y_obs, N_obs);
		// If the robot is in the line of sight, move away from the opponent's line of sight
		double defender_x = defender->x[2];
		double defender_y = defender->x[3];
		double opponent_x = S1->P[2]->x[2];
		double opponent_y = S1->P[2]->x[3];

		// Perpendicular direction calculation to avoid LoS
		double dx = opponent_x - defender_x;
		double dy = opponent_y - defender_y;

		double perpendicular_dx = -dy; // Rotate 90 degrees to the left
		double perpendicular_dy = dx; // Rotate 90 degrees to the left

		// Normalize the perpendicular direction
		double magnitude = sqrt(perpendicular_dx * perpendicular_dx + perpendicular_dy * perpendicular_dy);
		perpendicular_dx /= magnitude;
		perpendicular_dy /= magnitude;

		// Calculate the new target position based on the perpendicular vector
		double move_x = defender_x + perpendicular_dx * 100; // Move by 100 units in the perpendicular direction
		double move_y = defender_y + perpendicular_dy * 100;

		// Use the existing navigation function to move the robot
		navigate_to_target(defender, hide_x, hide_y, x_obs, y_obs, N_obs, pw_l, pw_r);
	}
	find_hiding_position(S1, hide_x, hide_y, hide_theta, x_obs, y_obs, N_obs);
	//navigate_to_target(defender, hide_x, hide_y, x_obs, y_obs, N_obs, pw_l, pw_r);
	last_opp_x = curr_opp_x;
	last_opp_y = curr_opp_y;
}


//Marc functions
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
				
                if (delta_l > delta_r) {
                    pw_l -= 200;
                    pw_r += 100;
                } else {
                    pw_l += 100;
                    pw_r -= 200;
                }

                return;
            }
        }
    }
}


//Jacob Functions

void Get_Object_RGB_Colour(image& rgb, int number_labels, vector<array<int, 5>>& Pos_RGB) {

	//Create Local Variables
	int i, j, Pixel_Number, k;
	int height, width;
	ibyte* p;

	//Initialize Variables
	height = rgb.height;
	width = rgb.width;
	p = rgb.pdata;

	cout << "\nUsing the following format: [Ic, Jc, R, G, B]";
	//Identify Colour of Each Object
	for (k = 1; k <= number_labels; k++) {
		for (i = 0; i < 3; i++) {

			Pixel_Number = (Pos_RGB[k][1] * width) + Pos_RGB[k][0];
			j = i + 2;
			Pos_RGB[k][j] = p[3 * Pixel_Number + i];
		}
		cout << "\n\nObject " << k << " has the follwing position and RGB code:";
		cout << "\nIc= " << (int)Pos_RGB[k][0];
		cout << "\nJc= " << (int)Pos_RGB[k][1];
		cout << "\nR= " << (int)Pos_RGB[k][4];
		cout << "\nG= " << (int)Pos_RGB[k][3];
		cout << "\nB= " << (int)Pos_RGB[k][2];
		draw_point_rgb(rgb, (int)Pos_RGB[k][0], (int)Pos_RGB[k][1], 255, 255, 255);
	}
}
