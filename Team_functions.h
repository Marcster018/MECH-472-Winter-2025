//mech 472 functions
#include <vector>
#include <array>

using namespace std;

//Attack
void Attack_Sequence(image& rgb, int& pw_r, int& pw_l, int& pw_laser, int& laser);

//Anthony functions
void find_hollow_circles(int& nlabels, image& rgb, image& label, image&a, image& rgb0, int Ic[4], int Jc[4])
void clean_up(image& rgb, image&a)
void find_obstacles(image& rgb, image& label, image& a, int nlabel, int ObsLabel[2])
void opponent_track(int Ic[4], int Jc[4], int Oc[2], image &rgb, int &pw_r, int &pw_l)
void detect_obstruction(int Ic[4], int Jc[4], int ObsLabel[2], image& rgb, image& label, bool &detected)
void go_to(int Ic[4], int Jc[4], int& pw_l, int& pw_r, image& rgb,image &label, int id,int jd)

//Fred functions
int auto_select_shape_by_size(i2byte& nlabel, image& label)
//dynamic_hide also requires find_hollow_circles and clean_up from Anthony's functions
int find_obstacles(image& rgb, image& label, image& a, int obs_x[], int obs_y[], int max_obs);
void find_hiding_position(int defender_x, int defender_y, int opponent_x, int opponent_y, image& rgb, image& label, image& a, double& hide_x, double& hide_y);
void navigate_to_target(robot* defender, double hide_x, double hide_y, image& rgb, image& label, image& a, int& pw_l, int& pw_r);
void dynamic_hide(robot* defender, image& rgb, image& rgb0, image& label, image& a, image& b, int& pw_l, int& pw_r);
bool is_robot_in_line_of_sight(int defender_x, int defender_y, int opponent_x, int opponent_y, image& rgb, image& label, image& a);
double estimate_radius_from_image(image& rgb_obstacle, double IC, double JC);

//marc functions
void Collision_Detection(robot* my_robot, image& label, int& pw_l, int& pw_r)

//Jacob Functions
static void BWProcessing(image& InputImage, image& OutputImage);
static void ColourProcessing(image& InputImage, image& OutputImage);
static void Process_Image(image& InputImage, image& LabelImageBW, image& LabelImageColour, int& nlabelBW, int& nlabelColour); //Takes an RGB image and returns a correctly labeled image
static void Get_Image_Data(image& rgb, image& LabelImageBW, image& LabelImageColour, int nlabelBW, int nlabelColour, vector<array<int, 5>>& Bulk_Data);//Takes an RGB image and a labeled image and returns data on each object in the format [Ic, Jc, theta, H, S, V]
static void Classify_Data();