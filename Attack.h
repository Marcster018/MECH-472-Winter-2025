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

void clean_up(image& rgb, image& a) {
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

void opponent_track(int Ic[4], int Jc[4], image& rgb, image& label, int& pw_r, int& pw_l) {

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

void detect_obstruction(int Ic[4], int Jc[4], int ObsLabel[2], image& rgb, image& label, bool& detected) {
	int width = rgb.width;
	int height = rgb.height;
	int size = width * height;
	bool obstruction = false;

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

void go_to(int Ic[4], int Jc[4], int& pw_l, int& pw_r, image& rgb, image& label, int id, int jd) {
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
		else if (x_error > 1 && y_error < 1) {
			pw_l = 1500;
			pw_r = 1500;
		}
	}

}
//make sure rgb,rgb0,label,a,b are properly defined in the main
void attack(image& rgb,image&rgb0, image&a,image&b, image& label, int &pw_r, int &pw_l) {
	int height, width, nlabel, Ic[4], Jc[4], Oc[2], Obs[2][2], ObsLabel[2];

	copy(rgb, rgb0);
	clean_up(rgb, a);
	label_image(a, label, nlabel);
	bool detected = false;
	find_hollow_circles(nlabel, rgb, label, a, rgb0, Ic, Jc);
	find_obstacles(rgb, label, a, nlabel, ObsLabel);
	detect_obstruction(Ic, Jc, ObsLabel, rgb, label, detected);

	if (detected == false) opponent_track(Ic, Jc, rgb, label, pw_r, pw_l);
	else {
		pw_l = 1350;
		pw_r = 1750;
		//here the patrol function should go
	}
}
