//mech 472 functions


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


//marc functions


//Jacob Functions
