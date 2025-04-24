//mech 472 functions

#include "Team_functions.h"
#include <vector>
#include <array>

using namespace std;

//Anthony functions
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

void opponent_track(int Ic[4], int Jc[4], int Oc[2], image &rgb, int &pw_r, int &pw_l) {
	
	int i = (Ic[3] + Ic[2])/2;
	int j = (Jc[3] + Jc[2])/2;
	
	Oc[0] = i;
	Oc[1] = j;
	int width;
	ibyte* p, *pc;
	width=rgb.width;

	//drawing line between red point and enemy
	//line is y=mx+b
	
	int dx = (Ic[0] - i);
	int dy = (Jc[0] - j);

	float m,b,y;

	if (dx == 0){
		m = 0;
		b = Ic[0];
	}
	else {
		m = (float)dy / (float)dx;
		b = Jc[0] - Ic[0] * m;
	}
	//cout << "\ny= " << m << " + " << b;

	//want to check if the red dot Ic[1], Jc[1] is bellow or above the line
	float deadzone=10;
	y = m * Ic[1] + b;

	if (i <= Ic[0]) {
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

	else if (i > Ic[0]) {
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
	draw_point_rgb(rgb, i, j, 0, 0, 255);

}

void find_obstacles(image& rgb, image& label, image&a,int nlabel,int Obs[2][2]) {
	int k, i, j, width, size, height, l, 
		area, max_area1=0, max_area2=0, max_label1=0, max_label2=0;
	double io1=0, jo1=0, io2=0, jo2=0;
	ibyte* p, * pc;
	i2byte* pl, * plc;

	width = rgb.width;
	height = rgb.height;
	size = width * height;

	p = rgb.pdata;
	pl = (i2byte*)label.pdata;

	for (l = 1; l < nlabel; l++) {
		area = 0;
		for (k = 0; k < size; k++) {
			plc = pl + k;

			if (*plc == l) {
				area++;
			}
		}
		if (area > max_area1) {
			max_area2 = max_area1;
			max_label2 = max_label1;
			io2 = io1;
			jo2 = jo1;

			max_area1 = area;
			max_label1 = l;
			centroid(a, label,l, io1, jo1);
		}
		else if (area > max_area2) {
			max_area2 = area;
			max_label2 = 2;
			centroid(a, label, l, io2, jo2);
		}
	}

	Obs[0][0] = (int)io1;
	Obs[0][1] = (int)jo1;
	Obs[1][0] = (int)io2;
	Obs[1][1] = (int)jo2;
	
	draw_point_rgb(rgb, (int)io1, (int)jo1, 255, 0, 0);
	draw_point_rgb(rgb, (int)io2, (int)jo2, 0, 255, 0);
	//cout <<"\nA1 = "<< max_area1<<" l1 = "<<max_label1<<" io = "<<io1<<" jo1 = "<<jo1;
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

//marc functions


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
