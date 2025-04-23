//mech 472 functions

#include "Team_functions.h"

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

			//Red
			if (R > 100 && G < 100 && B < 100) {
				draw_point_rgb(rgb, ic, jc, 255, 0, 0);
				Ic[0] = ic;
				Jc[0] = jc;
			}
			//green
			else if (R < 120 && G >120 && B < 150) {
				draw_point_rgb(rgb, ic, jc, 0, 255, 0);
				Ic[1] = ic;
				Jc[1] = jc;
			}
			//blue
			else if (R < 60 && G < 175 && B > 200) {
				draw_point_rgb(rgb, ic, jc, 0, 0, 255);
				Ic[2] = ic;
				Jc[2] = jc;
			}
			//orange
			else if (R > 220 && G > 130 && G < 200 && B < 150) {
				draw_point_rgb(rgb, ic, jc, 255, 100, 0);
				Ic[3] = ic;
				Jc[3] = jc;
			}
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

//marc functions


//Jacob Functions

void get_pixel_colour_rgb(image& rgb, int Ic, int Jc, int(&Colour)[3]) {

	//Create Local Variables
	int k;
	int height, width;
	ibyte* p;

	//Initialize Variables
	height = rgb.height;
	width = rgb.width;
	p = rgb.pdata;
	k = (Jc * width) + Ic;

	for (int i = 0; i < 3; i++) {
		Colour[i] = p[3 * k + i];
	}
}