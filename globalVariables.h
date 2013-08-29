#ifndef __GLOBALVARIABLES_H__
#define __GLOBALVARIABLES_H__

#include <vector>


/*******************************************************************************************************************/

//------------------------------------------
//Flags - enable or disable features
#define USE_EUCLIDEAN_DISTANCE_HEURISTIC true

#define EIGHT_CONNECTED_GRIDWORLD
//#define FOUR_CONNECTED_GRIDWORLD

//#define SHOW_DEBUG_INFO
//------------------------------------------

#define GRIDWORLD_ROWS 24
#define GRIDWORLD_COLS 29
#define MAX_INT 2147483647
#define SQRT2 1.41421356237
#define INF 999

using namespace std;

extern int numberOfExpandedStates;
extern int numberOfVertexAccesses;
extern bool change_occured;
extern string initGridworldFileName, changedGridworldFileName, outputGridworldFileName;

enum cellType{TRAVERSABLE='0', BLOCKED='1', START='6', GOAL='7', UNKNOWN='9'};
enum vertexStatus{UNEXPLORED='0', EXPANDED='1', ACCESSED='2'};

struct expandedVertex{
	 int row;
	 int col;
	 double g;
	 double f;
};

struct vertex
{
    double rhs;
    double g;
	 int c;
	 double h;
	 double f;
    char type; 
	 int row;
	 int col;
	 char status; 
	 double key1;
	 double key2;
	 vertex *pred;
	 int steps;
}; 

extern int MAX_MOVES;

extern vertex map[GRIDWORLD_ROWS][GRIDWORLD_COLS];
extern vertex map2[GRIDWORLD_ROWS][GRIDWORLD_COLS];
extern vertex* startVertex;
extern vertex* goalVertex;

typedef struct CellPosition
{
	int row;
	int col;
};

typedef struct vector<CellPosition> pathType;

/*******************************************************************************************************************/

extern int fieldX1, fieldY1, fieldX2, fieldY2; //playing field boundaries
extern float WORLD_MAXX;
extern float WORLD_MAXY;
// colour constants
extern int BACKGROUND_COLOUR;
extern int LINE_COLOUR;



#endif
