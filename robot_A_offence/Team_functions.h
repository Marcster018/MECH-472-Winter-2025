//mech 472 functions
#include <vector>
#include <array>

using namespace std;

//Attack
void Attack_Sequence(image& rgb, int& pw_r, int& pw_l, int& pw_laser, int& laser, char Player);

//Anthony functions
void find_hollow_circles(int& nlabels, image& label, image& a, image& rgb0, int Ic[4], int Jc[4]);
void clean_up(image& rgb, image& a);
void find_obstacles(image& rgb, image& label, image& a, int nlabel, vector<int>& OL);
void opponent_track(int Ic[4], int Jc[4], image& rgb, image& label, int& pw_r, int& pw_l, int&laser);
void detect_obstruction(int Ic[4], int Jc[4], vector<int>& OL, image& rgb, image& label, bool& detected);
void go_to(int Ic[4], int Jc[4], int& pw_l, int& pw_r, image& rgb, int id, int jd);
void quadrant(int Ic[4], int Jc[4], image& rgb, int& id, int& jd);
void find_path(int Ic[4], int Jc[4], vector<int>& OL, image& rgb, image& label, int& id, int& jd);

//Fred functions
//int auto_select_shape_by_size(i2byte& nlabel, image& label);
//dynamic_hide also requires find_hollow_circles and clean_up from Anthony's functions
//int find_obstacles(image& rgb, image& label, image& a, int obs_x[], int obs_y[], int max_obs);
//find_hiding_position(int defender_x, int defender_y, int opponent_x, int opponent_y, image& rgb, image& label, image& a, double& hide_x, double& hide_y);
//void navigate_to_target(robot* defender, double hide_x, double hide_y, image& rgb, image& label, image& a, int& pw_l, int& pw_r);
//void dynamic_hide(int defender, image& rgb, image& rgb0, image& label, image& a, image& b, int& pw_l, int& pw_r);
//bool is_robot_in_line_of_sight(int defender_x, int defender_y, int opponent_x, int opponent_y, image& rgb, image& label, image& a);
double estimate_radius_from_image(image& rgb_obstacle, double IC, double JC);

//marc functions
void Collision_Detection(image& rgb, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data, int& pw_l, int& pw_r);

//Jacob Functions
static bool ObstacleAtLocation(int I, int J, vector<array<int, 6>>& Obstacle_Data);
static void BWProcessing(image& InputImage, image& OutputImage);
static void ColourProcessing(image& InputImage, image& OutputImage);
static void Process_Image(image& InputImage, image& LabelImageBW, image& LabelImageColour, int& nlabelBW, int& nlabelColour); //Takes an RGB image and returns a correctly labeled image
static void Get_Image_Data(image& rgb, image& LabelImageBW, image& LabelImageColour, int nlabelBW, int nlabelColour, vector<array<int, 6>>& Bulk_Data);//Takes an RGB image and a labeled image and returns data on each object in the format [Ic, Jc, theta, H, S, V]
static bool Hollow_Circle(image& rgb, image& LabelImage, int& nlabels, int Ic, int Jc);
static enum ColourTypes { GREEN, RED, ORANGE, BLUE, WRONG }; //Possible robot colours. Each colour is assigned an integer for simplicity
static bool NotGrey(int R, int G, int B);
static ColourTypes Object_Colour(int R, int G, int B);
static double Calculate_Theta(double x1, double y1, double x2, double y2);
static void Assign_Robot_Data(char Player, ColourTypes Colour, array<int, 6>& Bulk_Data, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data);
static void Classify_Data(image& rgb, image& LabelImageBW, image& LabelImageColour, int nlabelBW, int nlabelColour, vector<array<int, 6>>& Bulk_Data, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data, char Player);
static void Classify_Data_Troubleshooting(image& rgb, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data);
static void Shoot_Laser(image& rgb, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data, int& pw_laser, int& laser);