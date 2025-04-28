//mech 472 functions
#include <vector>
#include <array>

using namespace std;

//Anthony functions
void find_hollow_circles(int& nlabels, image& rgb, image& label, image&a, image& rgb0, int Ic[4], int Jc[4])
void clean_up(image& rgb, image&a)
void find_obstacles(image& rgb, image& label, image& a, int nlabel, int ObsLabel[2])
void opponent_track(int Ic[4], int Jc[4], int Oc[2], image &rgb, int &pw_r, int &pw_l)
void detect_obstruction(int Ic[4], int Jc[4], int ObsLabel[2], image& rgb, image& label, bool &detected)
void go_to(int Ic[4], int Jc[4], int& pw_l, int& pw_r, image& rgb,image &label, int id,int jd)

//Fred functions
int auto_select_shape_by_size(i2byte& nlabel, image& label)

//marc functions

//Jacob Functions
void Get_Object_RGB_Colour(image& rgb, int number_labels, vector<array<int, 5>>& Pos_RGB)
