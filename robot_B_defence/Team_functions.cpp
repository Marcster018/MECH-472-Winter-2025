
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

//Anthony's functions

//finds location of ostacles
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
		if (diff < 3 && diff>0) {//by trial and error below 3 were the obstacles for diff
			OL.push_back(l);
			centroid(a, label, l, ic, jc);
			//draw_point_rgb(rgb, ic, jc, 255, 0, 0); //For Troubleshooting
		}
	}
}

//Frederique's functions

//Checks if the defender is in the opponent's line of sight
bool is_robot_in_line_of_sight(array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data) {
	double dx, dy, fx, fy, a, b, c, sqrt_quad, t1, t2;
	int defender_x, defender_y;
	int opponent_x, opponent_y;

	defender_x = (Robot_Data[0][0] + Robot_Data[1][0]) / 2;
	defender_y = (Robot_Data[0][1] + Robot_Data[1][1]) / 2;

	opponent_x = (Opponent_Data[0][0] + Opponent_Data[1][0]) / 2;
	opponent_y = (Opponent_Data[0][1] + Opponent_Data[1][1]) / 2;

	dx = defender_x - opponent_x;
	dy = defender_y - opponent_y;
	
	//check if the opponent line of sight is obstructed by an obstacle
	for (int i = 0; i < Obstacle_Data.size(); i++) {
		fx = Obstacle_Data[i][0] - opponent_x;
		fy = Obstacle_Data[i][1] - opponent_y;

		double radius = 80;

		a = dx * dx + dy * dy;
		b = 2 * (fx * dx + fy * dy);
		c = fx * fx + fy * fy - radius * radius;

		sqrt_quad = b * b - 4 * a * c;

		if (sqrt_quad >= 0) {
			
			sqrt_quad = sqrt(sqrt_quad);
			t1 = (-b - sqrt_quad) / (2 * a);
			t2 = (-b + sqrt_quad) / (2 * a);
	
			// If the line from robot to opponent intersects this obstacle
			if ((t1 >= 0 && t1<= 1) || (t2 >= 0 && t2 <= 1)) {
				return false;  // Line of sight is blocked
			}
			
		}

	}
	return true;
}

//Finds best possible hiding position for the defender
void find_hiding_position(array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data, double& hide_x, double& hide_y) {
	int defender_x, defender_y;
	int opponent_x, opponent_y;

	defender_x = (Robot_Data[0][0] + Robot_Data[1][0]) / 2;
	defender_y = (Robot_Data[0][1] + Robot_Data[1][1]) / 2;
	
	opponent_x = (Opponent_Data[0][0] + Opponent_Data[1][0]) / 2;
	opponent_y = (Opponent_Data[0][1] + Opponent_Data[1][1]) / 2;

	//cout << "\nopponent_x = " << opponent_x << "   opponent_y = " << opponent_y;

	// find a hiding spot
	double best_score = 1e6; 

	for (int i = 0; i < Obstacle_Data.size(); i++) {
		// Direction from opponent to obstacle
		double dx = Obstacle_Data[i][0] - opponent_x;
		double dy = Obstacle_Data[i][1] - opponent_y;
		double dist_to_opp_sq = dx * dx + dy * dy;

		// Candidate hiding position (behind obstacle relative to opponent)
		double candidate_x = Obstacle_Data[i][0] + dx * 0.5;
		double candidate_y = Obstacle_Data[i][1] + dy * 0.5;

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

//Makes the defender move to best possible hiding position
void navigate_to_target(robot* defender, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data, double hide_x, double hide_y, int& pw_l, int& pw_r) {
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
	for (int i = 0; i < Obstacle_Data.size(); i++) {
		double obs_dx = rx - Obstacle_Data[i][0];
		double obs_dy = ry - Obstacle_Data[i][1];
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

//Defence sequence moves both the opponent and the defender
void Defence_Sequence(robot* defender, image& rgb, int& pw_l, int& pw_r, char Player, int& pw_l_o, int& pw_r_o) {
	int height, width, nlabelBW, nlabelColour;
	static vector <array <int, 6>> Bulk_Data; //Using vector to be able to scale the number of rows to the number of objects
	static array<array<int, 6>, 2> Robot_Data; //Using array notation to be able to use the function of array a = array b instead of using pointer notation
	static array<array<int, 6>, 2> Opponent_Data;//Using array notation to be able to use the function of array a = array b instead of using pointer notation
	static vector <array <int, 6>> Obstacle_Data;//Using vector to be able to scale the number of rows to the number of objects
	bool detected = false;
	image ProcessingImage, LabelImageBW, LabelImageColour;

	ProcessingImage.type = RGB_IMAGE;
	ProcessingImage.width = rgb.width;
	ProcessingImage.height = rgb.height;

	LabelImageBW.type = LABEL_IMAGE;
	LabelImageBW.width = rgb.width;
	LabelImageBW.height = rgb.height;

	LabelImageColour.type = LABEL_IMAGE;
	LabelImageColour.width = rgb.width;
	LabelImageColour.height = rgb.height;

	allocate_image(LabelImageBW);
	allocate_image(LabelImageColour);
	allocate_image(ProcessingImage);

	copy(rgb, ProcessingImage);

	Process_Image(ProcessingImage, LabelImageBW, LabelImageColour, nlabelBW, nlabelColour);

	Get_Image_Data(rgb, LabelImageBW, LabelImageColour, nlabelBW, nlabelColour, Bulk_Data);

	Classify_Data(rgb, LabelImageBW, LabelImageColour, nlabelBW, nlabelColour, Bulk_Data, Robot_Data, Opponent_Data, Obstacle_Data, Player);

	static double hide_x = 0;
	static double hide_y = 0;

	static double last_opp_x = 0;
	static double last_opp_y = 0;

	bool exposed = is_robot_in_line_of_sight(Robot_Data, Opponent_Data, Obstacle_Data);
	
	int defender_x, defender_y;
	int opponent_x, opponent_y;

	defender_x = (Robot_Data[0][0] + Robot_Data[1][0]) / 2;
	defender_y = (Robot_Data[0][1] + Robot_Data[1][1]) / 2;

	opponent_x = (Opponent_Data[0][0] + Opponent_Data[1][0]) / 2;
	opponent_y = (Opponent_Data[0][1] + Opponent_Data[1][1]) / 2;

	opponent_loop_path(Opponent_Data, pw_l_o, pw_r_o);

	if (exposed) {
		find_hiding_position(Robot_Data, Opponent_Data, Obstacle_Data, hide_x, hide_y);
	}
	navigate_to_target(defender, Robot_Data, Opponent_Data, Obstacle_Data, hide_x, hide_y, pw_l, pw_r);
	
	last_opp_x = opponent_x;
	last_opp_y = opponent_y;

	free_image(LabelImageBW);
	free_image(LabelImageColour);
	free_image(ProcessingImage);
}

//Makes the opponent go in a rectangular path in a loop
void opponent_loop_path(array<array<int, 6>, 2>& Opponent_Data, int& pw_l_o, int& pw_r_o) {

	static int phase = 1;
	const double PI = 3.14159265358979323846;
	// Robot current position
	double opponent_x = (Opponent_Data[0][0] + Opponent_Data[1][0]) / 2;
	double opponent_y = (Opponent_Data[0][1] + Opponent_Data[1][1]) / 2;
	double rtheta = atan2(Opponent_Data[0][1] - Opponent_Data[1][1],
		Opponent_Data[0][0] - Opponent_Data[1][0]);

	// ---- PHASE 1: (150, 375) -> (480, 375)
	if (phase == 1) {
		double dx = 480 - opponent_x;
		double dy = 375 - opponent_y;
		double dist = sqrt(dx * dx + dy * dy);
		double angle_to_target = atan2(dy, dx);
		double angle_diff = angle_to_target - rtheta;

		while (angle_diff > PI) angle_diff -= 2 * PI;
		while (angle_diff < -PI) angle_diff += 2 * PI;
		cout << "\nangle = " << angle_diff;
		int turn = (int)(angle_diff * 500);
		if (dist > 40.0) {
			if (fabs(angle_diff) > 0.3) {
				pw_l_o = 1300 + turn;
				pw_r_o = 1700 + turn;
			}
			else {
				pw_l_o = 1000  ;
				pw_r_o = 2000  ;
			}
		}
		else {
			phase = 2;
		}
	}

	// ---- PHASE 2: (480, 375) -> (480, 80)
	else if (phase == 2) {
		double dx = 480 - opponent_x;
		double dy = 80 - opponent_y;
		double dist = sqrt(dx * dx + dy * dy);
		double angle_to_target = atan2(dy, dx);
		double angle_diff = angle_to_target - rtheta;

		while (angle_diff > PI) angle_diff -= 2 * PI;
		while (angle_diff < -PI) angle_diff += 2 * PI;
		int turn = (int)(angle_diff * 500);
		if (dist > 40.0) {
			if (fabs(angle_diff) > 0.3) {
				pw_l_o = 1300 + turn;
				pw_r_o = 1700 + turn;
			}
			else {
				pw_l_o = 1000  ;
				pw_r_o = 2000  ;
			}
		}
		else {
			phase = 3;
		}
	}

	// ---- PHASE 3: (480, 80) -> (60, 80)
	else if (phase == 3) {
		double dx = 60 - opponent_x;
		double dy = 80 - opponent_y;
		double dist = sqrt(dx * dx + dy * dy);
		double angle_to_target = atan2(dy, dx);
		double angle_diff = angle_to_target - rtheta;

		while (angle_diff > PI) angle_diff -= 2 * PI;
		while (angle_diff < -PI) angle_diff += 2 * PI;

		int turn = (int)(angle_diff * 500);
		if (dist > 40.0) {
			if (fabs(angle_diff) > 0.3) {
				pw_l_o = 1300 + turn;
				pw_r_o = 1700 + turn;
			}
			else {
				pw_l_o = 1000;
				pw_r_o = 2000;
			}
		}
		else {
			phase = 4;
		}
	}

	// ---- PHASE 4: (60, 80) -> (60, 480)
	else if (phase == 4) {
		double dx = 60 - opponent_x;
		double dy = 480 - opponent_y;
		double dist = sqrt(dx * dx + dy * dy);
		double angle_to_target = atan2(dy, dx);
		double angle_diff = angle_to_target - rtheta;

		while (angle_diff > PI) angle_diff -= 2 * PI;
		while (angle_diff < -PI) angle_diff += 2 * PI;

		int turn = (int)(angle_diff * 500);
		if (dist > 40.0) {
			if (fabs(angle_diff) > 0.2) {
				pw_l_o = 1300 + turn;
				pw_r_o = 1700 + turn;
			}
			else {
				pw_l_o = 1000;
				pw_r_o = 2000;
			}
		}
		else {
			phase = 1; // Loop back to phase 1
		}
	}
}


//Jacob Functions

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
		case GREEN:  Opponent_Data[0] = Bulk_Data; break; //Row 1 of Opponent_Data is equal to the current row of Bulk_Data
		case RED:    Opponent_Data[1] = Bulk_Data; break; //Row 2 of Opponent_Data is equal to the current row of Bulk_Data
		case ORANGE: Robot_Data[0] = Bulk_Data; break;
		case BLUE:   Robot_Data[1] = Bulk_Data; break;
		default: break;
		}
	}
	else if (Player == 'B') {
		switch (Colour) {
		case GREEN:  Robot_Data[0] = Bulk_Data; break;
		case RED:    Robot_Data[1] = Bulk_Data; break;
		case ORANGE: Opponent_Data[0] = Bulk_Data; break;
		case BLUE:   Opponent_Data[1] = Bulk_Data; break;
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
	vector<int> Obstacle_Objects; //1D dynamic array
	find_obstacles(rgb, LabelImageColour, TempImageA, nlabelColour, Obstacle_Objects);
	
	for (i = 0; i < Obstacle_Objects.size(); i++) {
		Obstacle_Objects[i] += nlabelBW; //Increment each label number by the number of BW labels since the BW labels come before the Colour labels in Bulk_Data
	}

	Obstacle_Data.resize(Obstacle_Objects.size());

	for (i = 0; i < Obstacle_Objects.size(); i++) {
		j = Obstacle_Objects[i] - 1;
		if (NotGrey(Bulk_Data[j][3], Bulk_Data[j][4], Bulk_Data[j][5])) {
			Obstacle_Data[accumulator] = Bulk_Data[j]; //Only update when object isn't grey
			accumulator++;
		}
	}
	Obstacle_Data.resize(accumulator);
	//Release image memory
	free_image(TempImageA);
	free_image(TempImageB);
}

void Classify_Data_Troubleshooting(image& rgb, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data) {
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