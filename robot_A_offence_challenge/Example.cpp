
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

void Example_Defence(image& rgb, int& pw_l_o, int& pw_r_o) {
	
}

void Example_Offence(image& rgb, int& pw_l_o, int& pw_r_o) {

}

void Example_Defence_Challenge(image& rgb, int& pw_l_o, int& pw_r_o) {

}

void Example_Offence_Challenge(image& rgb, int& pw_l_o, int& pw_r_o) {

}
//Controls a robot with arrow keys
//Pass by reference the pulse width variables of the desired robot
void Example_KeyboardInput(int& pw_l, int& pw_r) {
	if (KEY(VK_RIGHT)) {
		pw_r = 1300;
		pw_l = 1300;
	}
	if (KEY(VK_LEFT)) {
		pw_r = 1700;
		pw_l = 1700;
	}
	if (KEY(VK_UP)) {
		pw_r = 1800;
		pw_l = 1200;
	}
	if (KEY(VK_DOWN)) {
		pw_r = 1200;
		pw_l = 1800;
	}
}

