//@uthor: Asheesh Sharma
// An improvement constrined genetic algorithm for CVRP with hibrid BRBAX
// crossover and hibrid mution.
// The algorithm
// The algorithm has two sections.The population is initialized with popsize
// number of random chromosomes.
// The top five fittest individuals are crossed by using BRBAX operator and
// mutated with a certain probability.
// This produces four offsprings. The popsize is maintained by inserting
// popsize-4 randomly generated individual
// in the population after every iteration. Eventually, after some amount of
// iterations, the solution stops improving.
// This is probably when the solution is stuck in local maxima. To promote
// improvement, following mechanism
// is used.
// There algorithm described here uses a improvement constrained environment to
// avoid local maxima. There are two stages of
// avoiding it.

// 1. First stage is to save top five chromosomes with best fitness and flush
// all the rest. The population size is maintained by
// inserting popsize-5  randomly generated chromosomes in to the new population.
// The event takes place when the improvements are not
// made for a certain number of iteration cycles.

// 2. Second stage is when even after flushing the chromosomes as described in
// first stage, there is still no improvement. In such case
// the entire population is discarded and only the best solution is saved in a
// second population (which contains all the local best
// solutions so far). The event occurs when improvements are not made for
// certain percentage of iterations.

// This process ensures that the algorithm explores maximum number of local
// maxima and attain global minima.

// After finding some local maximas and storing them in a second population, the
// solution is further improved. This differs from the previos stage in two
// ways. First, the number of iterations is half less than the previous.
// Secondly, the cost is minimized rather than maximising the fitness.

// The Hibrid BRBAX cross over operator:
// The BRBAX operator works as follows.

// In general, the CVRP representation is translated to TSP before applying
// BRBAX. In this case, BRBAX is applied directly to the CVRP
// representation and the result is a TSP representaion. After applying a pure
// BRBAX, the TSP representation is optimized using 2_opt
// algorithm with certain probability. The 2_opt optimization is not applied in
// every BRBAX operation so as to maintain randomness
//(which will certainly converge to a same solution for any given offspring
//otherwise). Another reason for not applying 2_opt to every
// offspring is the fact that the it is known to get stuck in local maxima.

// The hibrid mutation operator
// The mutaion operator works as follows. In a first step, a route is mutated by
// swapping. Two nodes are randomly selected and swapped.
// In second stage, the same route is mutated by inversion. Two nodes are
// selected randomly (called the cutting nodes), and the order
// of nodes between them is inverted based on the follwing criteria. Only invert
// if, the sum of distances b/w (end cut node, neighbour
// of start cut node at position start_cut_node_pos-1) and (start cut node,
// neighbour of end cut node at position end_cut_node+1) is
// less than the sum of distances b/w (neighbour of start cut node at position
// start_cut_node_pos-1, start cut node) and (end cut
// node, neighbour of end cut node at position end_cut_node+1).
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <cstdlib>
#include <time.h>
#include <vector>
#include <iterator>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <functional>
#include <iomanip>
#include <deque>
#include <csignal>
using namespace std;
// Coordinate array
int coords[250][3] = {
    {1, -33, 33},    {2, -99, -97},   {3, -59, 50},    {4, 0, 14},
    {5, -17, -66},   {6, -69, -19},   {7, 31, 12},     {8, 5, -41},
    {9, -12, 10},    {10, -64, 70},   {11, -12, 85},   {12, -18, 64},
    {13, -77, -16},  {14, -53, 88},   {15, 83, -24},   {16, 41, 24},
    {17, 17, 21},    {18, 42, 96},    {19, -65, 0},    {20, -47, -26},
    {21, 85, 36},    {22, -35, -54},  {23, 54, -21},   {24, 64, 89},
    {25, 55, -17},   {26, 17, -25},   {27, -61, 66},   {28, -61, 26},
    {29, 17, -72},   {30, 79, 38},    {31, -62, -2},   {32, -90, -68},
    {33, 52, 66},    {34, -54, -50},  {35, 8, -84},    {36, 37, -90},
    {37, -83, 49},   {38, 35, -1},    {39, 7, 59},     {40, 12, 48},
    {41, 57, 95},    {42, 92, 28},    {43, -3, 97},    {44, -7, 52},
    {45, 42, -15},   {46, 77, -43},   {47, 59, -49},   {48, 25, 91},
    {49, 69, -14},   {50, -82, -19},  {51, 74, -70},   {52, 69, 59},
    {53, 29, 33},    {54, -97, 9},    {55, -58, 9},    {56, 28, 93},
    {57, 7, 73},     {58, -28, 73},   {59, -76, 55},   {60, 41, 42},
    {61, 92, 40},    {62, -84, -29},  {63, -12, 42},   {64, 51, -45},
    {65, -37, 46},   {66, -97, 35},   {67, 14, 89},    {68, 60, 58},
    {69, -63, -75},  {70, -18, 34},   {71, -46, -82},  {72, -86, -79},
    {73, -43, -30},  {74, -44, 7},    {75, -3, -20},   {76, 36, 41},
    {77, -30, -94},  {78, 79, -62},   {79, 51, 70},    {80, -61, -26},
    {81, 6, 94},     {82, -19, -62},  {83, -20, 51},   {84, -81, 37},
    {85, 7, 31},     {86, 52, 12},    {87, 83, -91},   {88, -7, -92},
    {89, 82, -74},   {90, -70, 85},   {91, -83, -30},  {92, 71, -61},
    {93, 85, 11},    {94, 66, -48},   {95, 78, -87},   {96, 9, -79},
    {97, -36, 4},    {98, 66, 39},    {99, 92, -79},   {100, -46, -17},
    {101, -30, -63}, {102, -42, 63},  {103, 20, 42},   {104, 15, 98},
    {105, 1, -17},   {106, 64, 20},   {107, -96, 85},  {108, 93, -29},
    {109, -40, -84}, {110, 86, 35},   {111, 91, 36},   {112, 62, -8},
    {113, -24, 4},   {114, 11, 96},   {115, -53, 62},  {116, -28, -71},
    {117, 7, -4},    {118, 95, -9},   {119, -3, 17},   {120, 53, -90},
    {121, 58, -19},  {122, -83, 84},  {123, -1, 49},   {124, -4, -3},
    {125, -82, 17},  {126, -43, 47},  {127, 6, -6},    {128, 70, 99},
    {129, 68, -29},  {130, -94, -30}, {131, -94, -20}, {132, -21, 77},
    {133, 64, 37},   {134, -70, -19}, {135, 88, 65},   {136, 2, 29},
    {137, 33, 57},   {138, -70, 6},   {139, -38, -56}, {140, -80, -95},
    {141, -5, -39},  {142, 8, -22},   {143, -61, -76}, {144, 76, -22},
    {145, 49, -71},  {146, -30, -68}, {147, 1, 34},    {148, 77, 79},
    {149, -58, -97}, {150, 82, 64},   {151, -80, 55},  {152, 81, -86},
    {153, 39, -49},  {154, -67, 72},  {155, -25, -89}, {156, -44, -95},
    {157, 32, -68},  {158, -17, 49},  {159, 93, 49},   {160, 99, 81},
    {161, 10, -49},  {162, 63, -41},  {163, 38, 39},   {164, -28, 39},
    {165, -2, -47},  {166, 38, 8},    {167, -42, -6},  {168, -67, 88},
    {169, 19, 93},   {170, 40, 27},   {171, -61, 56},  {172, 43, 33},
    {173, -18, -39}, {174, -69, -18}, {175, 75, 19},   {176, 31, 85},
    {177, 25, 58},   {178, -16, 36},  {179, 91, 15},   {180, 60, -39},
    {181, 49, -47},  {182, 42, 33},   {183, 16, -81},  {184, -78, 53},
    {185, 53, -80},  {186, -46, -26}, {187, -25, -54}, {188, 69, -46},
    {189, 0, -78},   {190, -84, 74},  {191, -16, 16},  {192, -63, -14},
    {193, 51, -77},  {194, -39, 61},  {195, 5, 97},    {196, -55, 39},
    {197, 70, -14},  {198, 0, 95},    {199, -45, -24}, {200, 38, 7},
    {201, 50, -37},  {202, 59, 71},   {203, -73, -96}, {204, -29, 72},
    {205, -47, 12},  {206, -88, -61}, {207, -88, 36},  {208, -46, -3},
    {209, 26, -37},  {210, -39, -67}, {211, 92, 27},   {212, -80, -31},
    {213, 93, -50},  {214, -20, -5},  {215, -22, 73},  {216, -4, -7},
    {217, 54, -48},  {218, -70, 39},  {219, 54, -82},  {220, 29, 41},
    {221, -87, 51},  {222, -96, -36}, {223, 49, 8},    {224, -5, 43},
    {225, -26, 54},  {226, -11, 60},  {227, 40, 61},   {228, 82, 35},
    {229, -92, 12},  {230, -93, -86}, {231, -66, 63},  {232, -72, -87},
    {233, -57, -84}, {234, 23, 52},   {235, -56, -62}, {236, -19, 59},
    {237, 63, -14},  {238, -13, 38},  {239, -19, 87},  {240, 44, -84},
    {241, 98, -17},  {242, -16, 62},  {243, 3, 66},    {244, 26, 22},
    {245, -38, -81}, {246, 70, 80},   {247, 17, -35},  {248, 96, -83},
    {249, -77, 44},  {250, -14, 80}};

int demands[250][2] = {
    {1, 0},    {2, 6},    {3, 72},   {4, 93},   {5, 78},    {6, 5},
    {7, 43},   {8, 1},    {9, 36},   {10, 28},  {11, 63},   {12, 25},
    {13, 50},  {14, 57},  {15, 1},   {16, 66},  {17, 37},   {18, 51},
    {19, 47},  {20, 53},  {21, 75},  {22, 48},  {23, 40},   {24, 8},
    {25, 69},  {26, 93},  {27, 29},  {28, 5},   {29, 53},   {30, 88},
    {31, 24},  {32, 53},  {33, 13},  {34, 47},  {35, 57},   {36, 9},
    {37, 74},  {38, 83},  {39, 96},  {40, 8},   {41, 80},   {42, 22},
    {43, 56},  {44, 43},  {45, 12},  {46, 73},  {47, 32},   {48, 8},
    {49, 79},  {50, 42},  {51, 4},   {52, 14},  {53, 17},   {54, 19},
    {55, 44},  {56, 5},   {57, 37},  {58, 100}, {59, 62},   {60, 79},
    {61, 57},  {62, 44},  {63, 37},  {64, 80},  {65, 60},   {66, 95},
    {67, 56},  {68, 56},  {69, 9},   {70, 90},  {71, 15},   {72, 4},
    {73, 58},  {74, 73},  {75, 5},   {76, 12},  {77, 3},    {78, 8},
    {79, 31},  {80, 39},  {81, 3},   {82, 52},  {83, 99},   {84, 29},
    {85, 12},  {86, 50},  {87, 98},  {88, 4},   {89, 56},   {90, 48},
    {91, 33},  {92, 45},  {93, 98},  {94, 4},   {95, 36},   {96, 72},
    {97, 26},  {98, 71},  {99, 84},  {100, 24}, {101, 99},  {102, 33},
    {103, 84}, {104, 74}, {105, 93}, {106, 25}, {107, 39},  {108, 42},
    {109, 77}, {110, 21}, {111, 50}, {112, 42}, {113, 71},  {114, 85},
    {115, 78}, {116, 64}, {117, 5},  {118, 93}, {119, 18},  {120, 68},
    {121, 29}, {122, 81}, {123, 4},  {124, 23}, {125, 11},  {126, 86},
    {127, 2},  {128, 31}, {129, 54}, {130, 38}, {131, 17},  {132, 81},
    {133, 72}, {134, 10}, {135, 50}, {136, 25}, {137, 71},  {138, 85},
    {139, 51}, {140, 87}, {141, 55}, {142, 45}, {143, 100}, {144, 38},
    {145, 11}, {146, 82}, {147, 50}, {148, 39}, {149, 6},   {150, 29},
    {151, 83}, {152, 22}, {153, 24}, {154, 69}, {155, 97},  {156, 65},
    {157, 97}, {158, 79}, {159, 79}, {160, 87}, {161, 52},  {162, 39},
    {163, 94}, {164, 97}, {165, 18}, {166, 3},  {167, 23},  {168, 19},
    {169, 40}, {170, 46}, {171, 96}, {172, 58}, {173, 15},  {174, 21},
    {175, 56}, {176, 67}, {177, 10}, {178, 36}, {179, 84},  {180, 49},
    {181, 85}, {182, 60}, {183, 33}, {184, 62}, {185, 70},  {186, 79},
    {187, 98}, {188, 99}, {189, 18}, {190, 59}, {191, 75},  {192, 94},
    {193, 89}, {194, 13}, {195, 19}, {196, 19}, {197, 90},  {198, 35},
    {199, 76}, {200, 55}, {201, 11}, {202, 98}, {203, 92},  {204, 1},
    {205, 2},  {206, 63}, {207, 57}, {208, 50}, {209, 19},  {210, 3},
    {211, 14}, {212, 18}, {213, 77}, {214, 28}, {215, 72},  {216, 49},
    {217, 58}, {218, 84}, {219, 58}, {220, 24}, {221, 98},  {222, 77},
    {223, 57}, {224, 39}, {225, 99}, {226, 83}, {227, 54},  {228, 86},
    {229, 2},  {230, 41}, {231, 42}, {232, 14}, {233, 55},  {234, 2},
    {235, 18}, {236, 17}, {237, 22}, {238, 28}, {239, 3},   {240, 14},
    {241, 53}, {242, 15}, {243, 36}, {244, 98}, {245, 96},  {246, 92},
    {247, 65}, {248, 64}, {249, 43}, {250, 50}};

double probability_mutation = 0.1;  // Probability of mutation
double probability_2_opt = 0.05;    // Probability of mutation

vector<vector<vector<int> > > chromosomes;  // Population vector

int done = 0;  // Conditional for 2_opt operations

// Calculate distance b/w two nodes.
double calc_dist(int node1, int node2) {
  // calculate cost for a pair of given nodes
  // cost is calulated by taking square root of (x2-x1)^2+(y2-y1)^2
  double x1, y1, x2, y2;
  double dist;
  x1 = coords[node1 - 1][1];
  y1 = coords[node1 - 1][2];
  x2 = coords[node2 - 1][1];
  y2 = coords[node2 - 1][2];
  dist = sqrtf(pow((x2 - x1), 2) + pow((y2 - y1), 2));
  return dist;
}

// get a total distance of given path
double get_path_distance(vector<int>& path) {
  double dist = 0.0;
  for (int i = 0; i < path.size() - 1; i++) {
    dist += calc_dist(path[i], path[i + 1]);
  }
  return dist;
}

// Calculate the demand of a give node.
int calc_demand(int node1) {
  int demand = demands[node1 - 1][1];
  return demand;
}

// Return a random number b/w 0 and a number.
int rands(int size) { return 0 + (std::rand() % (size - 0 + 1)); }

// Return the total demand of a path
int get_path_demand(vector<int>& path) {
  int totaldemand = 0;
  for (int i = 0; i < path.size(); i++) {
    totaldemand += calc_demand(path[i]);
  }
  return totaldemand;
}

// Check if a path is valid. Basically check if the total demand of path is less
// than or equal to the capacity of vehicle.
bool path_is_valid(vector<int>& path) { return get_path_demand(path) <= 500; }

// generate a random set of path (i.e a chromosome)
vector<vector<int> > generate_random_path() {
  vector<vector<int> > paths;  // An array of paths is a chormosome
  vector<int> path;            // An individual path is a gene
  vector<int> nodes;
  for (int i = 2; i <= sizeof(coords) / sizeof(coords[0]); i++) {
    nodes.push_back(i);
  }
  path.push_back(1);
  while (path_is_valid(path) && !nodes.empty()) {
    // Choose a random node from the nodes vector
    int get_node = rands(nodes.size() - 1);  // select a random index from nodes
    int i = nodes[get_node];                 // get its values
    path.push_back(i);                       // store it in the path
    if (path_is_valid(path)) {               // check if the path is valid
      // The path is valid, so we need to remove that node from the nodes
      nodes.erase(nodes.begin() + get_node);
    } else {
      path.pop_back();  // Remove the last item which was inserted outside the
      // if
      // statement
      path.push_back(1);      // the path is complete so end it with the depot
      paths.push_back(path);  // Add this valid path to the list of all paths
      path.clear();           // Empty the path so that it can be reused
      path.push_back(1);      // Reinitiate the path from the depot
    }
  }
  // The last path has to be stored so,
  path.push_back(1);
  paths.push_back(path);
  return paths;
}
// Find the closes neighbour to a given node in a path.
int closest_neihbour_in_list(int node1, vector<int>& array, int size) {
  double closest_dist = 100000000000;  // set the closest distance to be the
  // largest distance possible
  int closest_node = node1;
  for (int a = 0; a < array.size(); a++) {
    if (closest_dist >
        calc_dist(node1, array[a])) {  // if the current distance is greater
      // than the distance b/w this pair
      closest_dist =
          calc_dist(node1, array[a]);  // set the current distance to this one
      closest_node = array[a];         // set the closes node to be this one
    }
  }
  return closest_node;
}
// generate a random set of path (i.e a chromosome) based on closest neibour
vector<vector<int> > generate_random_path_based_closest_neighbour() {
  vector<vector<int> > paths;  // An array of paths is a chormosome
  vector<int> path;            // An individual path is a gene
  vector<int> nodes;
  int index_of_closest_city_in_v;
  for (int i = 2; i <= sizeof(coords) / sizeof(coords[0]); i++) {
    nodes.push_back(i);
  }
  path.push_back(1);
  while (path_is_valid(path) && !nodes.empty()) {
    // Choose a random node from the nodes vector
    index_of_closest_city_in_v =
        closest_neihbour_in_list(path.back(), nodes, nodes.size());

    int i = nodes[index_of_closest_city_in_v];  // get its values
    path.push_back(i);                          // store it in the path
    if (path_is_valid(path)) {                  // check if the path is valid
      // The path is valid, so we need to remove that node from the nodes
      nodes.erase(nodes.begin() + index_of_closest_city_in_v);
    } else {
      path.pop_back();  // Remove the last item which was inserted outside the
      // if
      // statement
      path.push_back(1);      // the path is complete so end it with the depot
      paths.push_back(path);  // Add this valid path to the list of all paths
      path.clear();           // Empty the path so that it can be reused
      path.push_back(1);      // Reinitiate the path from the depot
    }
  }
  // The last path has to be stored so,
  path.push_back(1);
  paths.push_back(path);
  return paths;
}
void generate_random_paths(bool normal_random_paths, int popsize) {
  vector<vector<int> > paths;
  if (normal_random_paths) {
    for (int j = 0; j < popsize; j++) {
      paths = generate_random_path();
      chromosomes.push_back(paths);
    }
  } else {
    for (int j = 0; j < 10; j++) {
      paths = generate_random_path_based_closest_neighbour();
      chromosomes.push_back(paths);
    }
  }
}

// Make path valid. it is the same as generate random paths. The difference is,
// the nodes are populated accordin to a given path.
bool is_node_in_path(int node, vector<int> nodes) {
  for (int i = 0; i < nodes.size(); i++) {
    if (node == nodes[i]) {
      return true;
    }
  }
  return false;
}

vector<vector<int> > make_valid_paths_from_a_path(vector<int> nodes) {
  vector<int> nodes_not_in_path;
  for (int i = 2; i <= sizeof(coords) / sizeof(coords[0]); i++) {
    if (!is_node_in_path(i, nodes)) {
      nodes_not_in_path.push_back(i);
    }
  }

  vector<vector<int> > paths;  // An array of paths is a chormosome
  vector<int> path;
  path.push_back(1);
  while (path_is_valid(path) && !nodes.empty()) {
    // Choose a random node from the nodes vector
    int get_node = rands(nodes.size() - 1);  // select a random index from nodes
    int i = nodes[get_node];                 // get its values
    path.push_back(i);                       // store it in the path
    if (path_is_valid(path)) {               // check if the path is valid
      // The path is valid, so we need to remove that node from the nodes
      nodes.erase(nodes.begin() + get_node);
    } else {
      path.pop_back();  // Remove the last item which was inserted outside the
      // if
      // statement
      path.push_back(1);      // the path is complete so end it with the depot
      paths.push_back(path);  // Add this valid path to the list of all paths
      path.clear();           // Empty the path so that it can be reused
      path.push_back(1);      // Reinitiate the path from the depot
    }
  }

  // Store the last path as well
  path.push_back(1);
  paths.push_back(path);

  if (nodes.empty() && !nodes_not_in_path.empty()) {
    path.clear();
    path.push_back(1);
    while (path_is_valid(path) && !nodes_not_in_path.empty()) {
      // Choose a random node from the nodes vector
      int get_node = rands(nodes_not_in_path.size() -
                           1);              // select a random index from nodes
      int i = nodes_not_in_path[get_node];  // get its values
      path.push_back(i);                    // store it in the path
      if (path_is_valid(path)) {            // check if the path is valid
        // The path is valid, so we need to remove that node from the nodes
        nodes_not_in_path.erase(nodes_not_in_path.begin() + get_node);
      } else {
        path.pop_back();  // Remove the last item which was inserted outside the
        // if
        // statement
        path.push_back(1);      // the path is complete so end it with the depot
        paths.push_back(path);  // Add this valid path to the list of all paths
        path.clear();           // Empty the path so that it can be reused
        path.push_back(1);      // Reinitiate the path from the depot
      }
    }

    // Store the last path as well
    path.push_back(1);
    paths.push_back(path);
  }

  // print_paths(paths);
  // cout<<"------"<<endl;
  return paths;
}

vector<vector<int> > make_valid_paths_from_a_path_BRBAX(vector<int> nodes) {
  vector<vector<int> > paths;  // An array of paths is a chormosome
  vector<int> path;
  path.push_back(1);
  while (path_is_valid(path) && !nodes.empty()) {
    // Choose a random node from the nodes vector
    int get_node = 0;           // select a node from nodes
    int i = nodes[get_node];    // get its values
    path.push_back(i);          // store it in the path
    if (path_is_valid(path)) {  // check if the path is valid
      // The path is valid, so we need to remove that node from the nodes
      nodes.erase(nodes.begin() + get_node);
    } else {
      path.pop_back();  // Remove the last item which was inserted outside the
      // if
      // statement
      path.push_back(1);      // the path is complete so end it with the depot
      paths.push_back(path);  // Add this valid path to the list of all paths
      path.clear();           // Empty the path so that it can be reused
      path.push_back(1);      // Reinitiate the path from the depot
    }
  }

  // Store the last path as well
  path.push_back(1);

  paths.push_back(path);
  // print_paths(paths);
  return paths;
}

vector<int> CVRP_to_TSP(vector<vector<int> > parent_CVRP) {
  vector<int> tmp_path;
  vector<int> parent_TSP;
  for (int i = 0; i < parent_CVRP.size(); i++) {
    tmp_path = parent_CVRP.at(i);
    for (int j = 0; j < tmp_path.size() - 1; j++) {
      parent_TSP.push_back(tmp_path.at(j));
    }
  }
  parent_TSP.push_back(1);
  return parent_TSP;
}

double total_cost_chromosome(vector<vector<int> > paths) {
  vector<int> path;
  double total_cost;
  for (int i = 0; i <= paths.size() - 1; i++) {
    path = paths.at(i);
    total_cost = total_cost + get_path_distance(path);
  }
  return total_cost;
}

// Calculate the fitness of a given path array
// Still have to work on it. Something doesnot look good.
double fitness_function(vector<vector<int> > paths) {
  double weight = 0.0;
  double avg_num_nodes;
  for (int i = 0; i < paths.size(); i++) {
    vector<int> path = paths[i];
    weight += get_path_distance(path);
    avg_num_nodes += path.size();
  }
  double cost = total_cost_chromosome(paths);
  avg_num_nodes = avg_num_nodes / paths.size();
  return weight;
}

// This is the mutation function it a modified mutation operator. Firstly, it
// swaps randomly seleted nodes(Allilis). Then inverts the order of nodes b/w a
// randomly selected range of nodes.

void inversion(vector<int>& path) {
  if (path.size() >= 4) {
    int end_node = 1 + (rand() % (int)((path.size() - 2) - 1 +
                                       1));  // End node of order inversion
    int start_node = 1 + (rand() % (int)(end_node - 1 + 1));  // Start node of
    // order inversion
    // should be less
    // than end_node

    int previous_neighbour_of_start_node = start_node - 1;
    int next_neighbour_of_end_node = end_node + 1;
    double d1 = calc_dist(path[previous_neighbour_of_start_node],
                          path[end_node]);  // d1 is distance b/w
    // previous_neighbour_of_start_node
    // and end_node
    double d2 = calc_dist(
        path[start_node],
        path[next_neighbour_of_end_node]);  // d2 is distance b/w start_node and
    // next_neighbour_of_end_node
    double d3 = calc_dist(
        path[start_node],
        path[previous_neighbour_of_start_node]);  // d3 is distance b/w
    // start_node and
    // previous_neighbour_of_start_node
    double d4 = calc_dist(
        path[end_node],
        path[next_neighbour_of_end_node]);  // d4 is distance b/w end_node and
    // next_neighbour_of_end_node

    if ((d1 + d2) < (d3 + d4)) {
      reverse(path.begin() + start_node,
              path.begin() + end_node);  // reverse the order of inversion
    }
  }
}

vector<int> mutate(vector<int> in_path) {
  int max = in_path.size() - 2;
  int min = 1;
  double numerator = static_cast<double>(rand());
  double denominator = static_cast<double>(RAND_MAX);
  double r = numerator / denominator;
  // Generate a random number b/w 0 and 1.
  for (int i = 1; i < in_path.size() - 1; i++) {
    if (r < probability_mutation &&
        in_path.size() > 5) {  // check if the mutation should be done
      // We are swapping i
      int swap_with = min + rand() % (max - min);  // Generate a random node to
      // swap with and ignore the last node (which is the depot)
      iter_swap(in_path.begin() + i,
                in_path.begin() + swap_with);  // Swap nodes
    }
    numerator = static_cast<double>(rand());
    denominator = static_cast<double>(RAND_MAX);
    r = numerator / denominator;  // Generate a random number b/w 0 and 1.
    inversion(in_path);           // Invert the order of nodes b/w two nodes.
  }
  return in_path;
}

vector<vector<int> > strip_depots_from_path(vector<vector<int> > paths) {
  vector<vector<int> > chromosome;
  vector<int> tmp_path;
  for (int i = 0; i < paths.size(); i++) {
    for (int j = 1; j < paths[i].size() - 1; j++) {
      tmp_path.push_back(paths[i][j]);
    }
    chromosome.push_back(tmp_path);
    tmp_path.clear();
  }

  return chromosome;
}

vector<int> sort_chrmosomes_based_on_fitness(
    vector<vector<vector<int> > > chromosome) {
  vector<double> fitness_index;
  vector<int> tops;

  for (int i = 0; i <= chromosome.size() - 1; i++) {
    fitness_index.push_back(fitness_function(chromosome.at(i)));
  }
  int index_of_element;
  for (int i = 0; i <= fitness_index.size() - 1 && i < 10; i++) {
    index_of_element = min_element(fitness_index.begin(), fitness_index.end()) -
                       fitness_index.begin();
    tops.push_back(index_of_element);
    fitness_index.erase(fitness_index.begin() + index_of_element);
  }

  return tops;
}

vector<int> sort_chrmosomes_based_on_cost(
    vector<vector<vector<int> > > chromosome) {
  vector<double> fitness_index;
  vector<int> tops;

  for (int i = 0; i <= chromosome.size() - 1; i++) {
    fitness_index.push_back(total_cost_chromosome(chromosome.at(i)));
  }
  int index_of_element;
  for (int i = 0; i <= fitness_index.size() - 1 && i < 10; i++) {
    index_of_element = min_element(fitness_index.begin(), fitness_index.end()) -
                       fitness_index.begin();
    tops.push_back(index_of_element);
    fitness_index.erase(fitness_index.begin() + index_of_element);
  }

  return tops;
}

void print_results(vector<vector<int> > paths) {
  ostringstream cmdstr;
  ostringstream tmp_cmdstr;
  ofstream file;
  cmdstr.precision(10);
  tmp_cmdstr.precision(10);
  double total_cost = 0;
  vector<int> path;
  cmdstr << "login as16542 33074\n";
  cmdstr << "name Asheesh Sharma\n";
  cmdstr << "algorithm An Iteration constrained Genetic Algorithm with hybrid mutation, "
            "2-opt and BRBAX crossover";

  total_cost = total_cost_chromosome(paths);
  cmdstr << "\ncost " << total_cost << "\n";

  for (int i = 0; i <= paths.size() - 1; i++) {
    path = paths.at(i);
    for (int j = 0; j <= path.size() - 1; j++) {
      if (j < path.size() - 1) {
        tmp_cmdstr << path.at(j) << "->";
      } else {
        tmp_cmdstr << path.at(j);
      }
    }
    cmdstr << tmp_cmdstr.str() << endl;
    tmp_cmdstr.str("");
  }
  cout << cmdstr.str();
  // file.open("best-solution.txt");
  file << cmdstr.str();
  file.close();
}

vector<vector<int> > sort_parent(vector<vector<int> > parent_CVRP) {
  vector<double> indices;
  vector<double> unsorted_indices;
  vector<vector<int> > tmp_CVRP;
  vector<double>::iterator it;
  for (int i = 0; i < parent_CVRP.size(); i++) {
    indices.push_back(
        static_cast<double>(500 - get_path_demand(parent_CVRP.at(i))) +
        static_cast<double>(rand()) /
            static_cast<double>(RAND_MAX));  // a random value is added so as to
                                             // make every index unique
  }
  unsorted_indices = indices;  // store unsorted paths
  sort(indices.begin(), indices.end());  // sort paths

  // find the sorted position of an element in unsorted indices which correspond
  // to the position of element in the parent_CVRP.
  for (int i = 0; i < indices.size(); i++) {
    it = find(unsorted_indices.begin(), unsorted_indices.end(), indices[i]);

    if (it != unsorted_indices.end()) {
      int pos = distance(unsorted_indices.begin(), it);
      // cout<<"FOUND  "<< *it<<"  at position: "<<pos<<endl;
      tmp_CVRP.push_back(parent_CVRP[pos]);
    }
  }

  parent_CVRP = tmp_CVRP;
  return parent_CVRP;
}

vector<int> TwoOptSwap(const int& i, const int& k, vector<int> path) {
  int size = path.size();
  vector<int> new_path(size);
  // 1. take route[0] to route[i-1] and add them in order to new_route
  for (int c = 0; c <= i - 1; ++c) {
    new_path[c] = path[c];
  }

  // 2. take route[i] to route[k] and add them in reverse order to new_route
  int dec = 0;
  for (int c = i; c <= k; ++c) {
    new_path[c] = path[k - dec];
    dec++;
  }

  // 3. take route[k+1] to end and add them in order to new_route
  for (int c = k + 1; c < size; ++c) {
    new_path[c] = path[c];
  }
  return new_path;
}

vector<int> two_opt_exchange(vector<int> path) {
  vector<int> tmp_path;
  int size = path.size();
  vector<int> tmp;

  double numerator = static_cast<double>(rand());
  double denominator = static_cast<double>(RAND_MAX);
  double r = numerator / denominator;
  // Generate a random number b/w 0 and 1.
  if (r < 0.6){
   tmp.push_back(path[0]);
   path.erase(path.begin());
   size = path.size();
   for (int i = 0; i < size; i++) {
     vector<int> excluded = path;
     std::vector<int>::iterator newEnd =
     std::remove(excluded.begin(), excluded.end(), tmp.back());
     excluded.erase(newEnd, excluded.end());
     int node = closest_neihbour_in_list(tmp.back(), excluded, excluded.size());
     tmp.push_back(node);

     std::vector<int>::iterator nEnd =
         std::remove(path.begin(), path.end(), node);
     path.erase(nEnd, path.end());
   }
   path = tmp;
   tmp.clear();
   size = path.size();
  }
  // repeat until no improvement is made
  int no_improve = 0;
  int improve = 0;
  while (no_improve <= 20) {
    if (improve > 50) {
      break;
    }
    double best_distance = get_path_distance(path);
    for (int i = 0; i < size - 1; i++) {
      for (int k = i + 1; k < size; k++) {
        tmp_path = TwoOptSwap(i, k, path);

        double new_distance = get_path_distance(tmp_path);

        if (new_distance < best_distance) {
          // Improvement
          path = tmp_path;
          best_distance = new_distance;
          no_improve++;
        } else {
          improve++;
        }
      }
    }
  }
  return path;
}

void end_opt(int signum) {
  cout << "\nOut of time\n";
  done = 1;
}

// 2-opt neighbor solution check
vector<int> two_change_path(vector<int> path) {
  vector<int> tmp_path;
  vector<int> tmp;
  tmp_path = path;
  // Get tour size
  int size = tmp_path.size();

  // repeat until no improvement is made
  int improve = 0;

  while (improve < 20) {
    double best_distance = get_path_distance(tmp_path);

    for (int i = 0; i < size - 1; i++) {
      for (int k = i + 1; k < size; k++) {
        tmp = TwoOptSwap(i, k, tmp_path);

        double new_distance = get_path_distance(tmp);

        if (new_distance < best_distance) {
          // Improvement found so reset
          improve = 0;
          tmp_path = tmp;
          best_distance = new_distance;
        }
      }
    }

    improve++;
  }

  return tmp_path;
}

bool is_chromosome_valid_TSP(vector<int> path) {
  if (path.size() == 249) {
    cout << "true" << endl;
    return true;
  } else {
    cout << "False:" << path.size() << endl;
    return false;
  }
}

bool is_chromosome_valid_CVRP(vector<vector<int> > chromosome) {
  int size = 0;
  for (int i = 0; i <= chromosome.size() - 1; i++) {
    vector<int> path = chromosome[i];
    size = size + path.size() - 2;
  }

  if (size == 249) {
    cout << "true" << endl;
    return true;
  } else {
    cout << "False:" << size << endl;
    return false;
  }
}
void print_paths(vector<vector<int> > paths) {
  cout << std::setprecision(16);
  ostringstream cmdstr;
  ostringstream tmp_cmdstr;
  cmdstr.precision(10);
  tmp_cmdstr.precision(10);
  vector<int> path;
  for (int i = 0; i <= paths.size() - 1; i++) {
    path = paths.at(i);
    for (int j = 0; j <= path.size() - 1; j++) {
      if (j < path.size() - 1) {
        tmp_cmdstr << path.at(j) << "->";
      } else {
        tmp_cmdstr << path.at(j);
      }
    }
    cmdstr << tmp_cmdstr.str() << endl;
    tmp_cmdstr.str("");
  }
  cout << cmdstr.str();
}
void brbax(vector<int> indices) {
  vector<vector<vector<int> > > old_chromosomes;
  vector<vector<int> > tmp_paths;
  vector<vector<int> > final_offspring;
  int num_nodes = sizeof(coords) / sizeof(coords[0]);
  for (int j = 0; j < indices.size() && j < 6; j++) {
    tmp_paths = chromosomes[indices[j]];
    old_chromosomes.push_back(tmp_paths);
  }
  chromosomes.clear();
  tmp_paths.clear();
  for (int k = 0; k < old_chromosomes.size() - 1; k++) {
    final_offspring.clear();
    vector<vector<int> > parent_CVRP_1 = old_chromosomes[k];
    vector<vector<int> > parent_CVRP_2 = old_chromosomes[k + 1];

    int num_routes_1 = parent_CVRP_1.size();
    int num_routes_2 = parent_CVRP_2.size();

    vector<vector<int> > offspring;

    // sort routes before translating CVRP to TSP
    // Sort the route vector in a (capacity - qty_supplied) ascending
    // fashion, i.e. lowest (capacity - qty_supplied) values first.
    parent_CVRP_1 = sort_parent(parent_CVRP_1);  // Sort parent 1
    parent_CVRP_2 = sort_parent(parent_CVRP_2);  // Sort parent 2

    // Array which marks the nodes already existent on the offspring.
    // If used_node(i) == FALSE, the station with index i is available for
    // insertion in the offspring.
    vector<bool> used_nodes;
    // As a starting point, all stations are available.
    for (int i = 1; i < num_nodes + 1; i++) {
      used_nodes.push_back(false);
    }
    // populate offspring with parent_1
    int j = 0;
    vector<int> offspring_path_TSP;
    for (int i = 0; i < (num_routes_1 / 2); i++) {
      vector<int> tmp_path;
      tmp_path = parent_CVRP_1[i];
      for (int j = 1; j < tmp_path.size() - 1; j++) {
        int curr_station = tmp_path[j];
        offspring_path_TSP.push_back(curr_station);
        used_nodes[tmp_path[j]] = true;
      }
    }
    // Get the rest of the nodes while preserving the order from parent 2
    for (int i = 0; i < parent_CVRP_2.size(); i++) {
      vector<int> tmp_path;
      tmp_path = parent_CVRP_2[i];
      for (int j = 1; j < tmp_path.size() - 1; j++) {
        int curr_station = tmp_path[j];
        if (!used_nodes[curr_station]) {
          offspring_path_TSP.push_back(curr_station);
          used_nodes[curr_station] = true;
        }
      }
    }

    // Apply 2_opt as a mutation
    double numerator = static_cast<double>(rand());
    double denominator = static_cast<double>(RAND_MAX);
    double r = numerator / denominator;
    if (r < probability_2_opt) {
      offspring_path_TSP = two_opt_exchange(offspring_path_TSP);
    }
    // build chromosome from new paths
    vector<vector<int> > offspring_path_CVRP =
        make_valid_paths_from_a_path_BRBAX(
            offspring_path_TSP);  // Convert the representation of offspring
                                  // from TSP to CVRP

    // Mutation on paths*/
    vector<int> tmp_path3;
    vector<int> tmp_path;
    vector<int> tmp_path1;
    for (int i = 0; i < offspring_path_CVRP.size(); i++) {
      tmp_path1 = offspring_path_CVRP[i];
      if (tmp_path1.size() > 5) {
        for (j = 1; j < tmp_path1.size() - 1; j++) {
          tmp_path.push_back(tmp_path1[j]);
        }
        tmp_path = two_change_path(tmp_path);

        tmp_path3.push_back(1);
        for (j = 0; j < tmp_path.size(); j++) {
          tmp_path3.push_back(tmp_path[j]);
        }
        tmp_path3.push_back(1);
      } else {
        tmp_path3 = tmp_path1;
      }
      final_offspring.push_back(tmp_path3);
      tmp_path3.clear();
      tmp_path.clear();
      tmp_path1.clear();
    }
    chromosomes.push_back(final_offspring);
  }
}

// Print paths (i.e a chromosome)

int main(int argc, char** argv) {
  // parameters iterations 150, flush 10%, zeroD 5
  // initialization
  srand(time(NULL));
  clock_t tStart = clock();
  int iterations = 500;
  int flush_percentage = 6;
  vector<vector<vector<int> > > best_chromosomes;  // best chromosomes saved at
                                                   // zero improvement iteration
                                                   // before flushing the
                                                   // population. This is to
                                                   // prevent local maximas.
  int pop_size = 512;

  vector<vector<vector<int> > > tmp_chromosomes;
  vector<vector<int> > best_path = generate_random_path();

  generate_random_paths(true, pop_size);
  vector<int> indices;

  int zeroDeltaIterations = 1;
  double prevWeight;

  for (int i = 0; i < iterations; i++) {
    double previous_fitness = fitness_function(best_path);
    double current_fitness;
    double previous_cost;
    double current_cost;
    indices = sort_chrmosomes_based_on_fitness(chromosomes);
    brbax(indices);
    generate_random_paths(true, pop_size - chromosomes.size());

    indices = sort_chrmosomes_based_on_fitness(chromosomes);
    vector<vector<int> > tmp_paths = chromosomes.at(indices[0]);

    if (fitness_function(tmp_paths) < fitness_function(best_path)) {
      best_path = tmp_paths;
      current_fitness = fitness_function(best_path);
      current_cost = total_cost_chromosome(best_path);
      zeroDeltaIterations = 1;
      probability_2_opt = probability_2_opt - 0.5;
      best_chromosomes.push_back(best_path);
    } else {
      probability_2_opt = probability_2_opt + 0.05;
      zeroDeltaIterations++;
    }

    if (zeroDeltaIterations * 100 / iterations >= flush_percentage) {
      // if no improvement was made for more than flush_percentage of
      // iterations, we are struck in local minima
      chromosomes.clear();
      generate_random_paths(true, pop_size);
      best_path = generate_random_path();
      // cout<<"here1: "<<zeroDeltaIterations*100/iterations<<"%"<<" of
      // "<<i<<endl;
      probability_2_opt = 0.7;
      zeroDeltaIterations = 1;
    } else if (zeroDeltaIterations % 10 == 0 && zeroDeltaIterations >= 10) {
      indices.clear();
      vector<int> indices = sort_chrmosomes_based_on_fitness(chromosomes);
      // Save Top 10 chromosomes so far and discard the rest
      for (int i = 0; i < 11; i++) {
        tmp_chromosomes.push_back(chromosomes[indices[i]]);
      }
      chromosomes.clear();
      chromosomes = tmp_chromosomes;
      generate_random_paths(true, pop_size - chromosomes.size());
      tmp_chromosomes.clear();
      probability_2_opt = probability_2_opt + 0.05;
    }
    if (i == iterations - 1) {
      best_chromosomes.push_back(best_path);
    }
    indices.clear();
    // cout << (double)(clock() - tStart) / CLOCKS_PER_SEC << " " <<
    // total_cost_chromosome(best_path)<<" "<<i<< endl;
  }

  indices.clear();
  indices = sort_chrmosomes_based_on_cost(best_chromosomes);
  best_path = best_chromosomes[indices[0]];
  //printf("\nTime taken: %.2fs\n\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  print_results(best_path);
  return 0;
}
