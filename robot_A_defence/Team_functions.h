//mech 472 functions
#include <vector>
#include <array>

using namespace std;

//Anthony's functions
void find_obstacles(image& rgb, image& label, image& a, int nlabel, vector<int>& OL);

//Frederique's functions
void find_hiding_position(array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data, double& hide_x, double& hide_y);
void navigate_to_target(robot* defender, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data, double hide_x, double hide_y, int& pw_l, int& pw_r);
void Defence_Sequence(robot* defender, image& rgb, int& pw_l, int& pw_r, char Player, int& pw_l_o, int& pw_r_o);
bool is_robot_in_line_of_sight(array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data);
void opponent_loop_path(array<array<int, 6>, 2>& Opponent_Data, int& pw_l_o, int& pw_r_o);


//Jacob Functions
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
void Classify_Data_Troubleshooting(image& rgb, array<array<int, 6>, 2>& Robot_Data, array<array<int, 6>, 2>& Opponent_Data, vector<array<int, 6>>& Obstacle_Data);