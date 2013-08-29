//v.1.4
///////////////////////////////////////////////////////////////////////////////////////////
//
//	 	      Program Name: Path-Planning algorithms
//	 	       Description: A*, LPA*, D* Lite
//
//	 		        History:  date of revision
//                         24th August 2013  
//
//      Start-up code by:  n.h.reyes@massey.ac.nz
//				 revision by:	Toby Herbert
///////////////////////////////////////////////////////////////////////////////////////////

#include <windows.h>
#include <ctype.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <deque>
#include <set>
#include <vector>

//-------------------------
#include "globalVariables.h"
#include "transform.h"
#include "graphics.h"
#include "algorithm.h"
#include "searchUtility.h"
#include "AstarSearch.h"
#include "LPAstarSearch.h"

using namespace std;

int numberOfExpandedStates;
int numberOfVertexAccesses;
string initGridworldFileName, changedGridworldFileName, outputGridworldFileName;

int MAX_MOVES;
vertex map[GRIDWORLD_ROWS][GRIDWORLD_COLS];
vertex map2[GRIDWORLD_ROWS][GRIDWORLD_COLS];
vertex* startVertex;
vertex* goalVertex;

float WORLD_MAXX;
float WORLD_MAXY;
int fieldX1, fieldY1, fieldX2, fieldY2;
WorldBoundaryType worldBoundary;
DevBoundaryType deviceBoundary;

int BACKGROUND_COLOUR;
int LINE_COLOUR;

int robotWidth;

void markWayPoint_RowCol(int color,int row, int col, int totalRows, int totalColumns);
void drawCell_RowCol(int color,int row, int col, int totalRows, int totalColumns, char* text);
void loadAndDisplayMap(const char* fn, vertex map[GRIDWORLD_ROWS][GRIDWORLD_COLS], int totalRows, int totalColumns);
void drawPathCell_RowCol(int color,int row, int col, int totalRows, int totalColumns, char* text);


void initSystemOfCoordinates(int &xInc, int &yInc){
	 int halfCentimeter = 2; //2 pixels
	 int oneCentimeter =  halfCentimeter*2;
	 int width, height;
	
	 int centerX, centerY;
	 
	 centerX = getmaxx()/2;
	 centerY = getmaxy()/2;
	
	 WORLD_MAXX = 220.0f;
    WORLD_MAXY = 180.0f;
	
	 width = oneCentimeter * int(WORLD_MAXX);
	 height = oneCentimeter * int(WORLD_MAXY);
	
	//-----------------------------
	 //left_top aligned across the screen
	 fieldX1 = (textwidth("W")*3);
    fieldX2 = (textwidth("W")*3) + width;
    fieldY1 = (textheight("W")*1);
    fieldY2 = (textheight("W")*3) + height;
	 //-----------------------------
	
	 //~ //-----------------------------
	 //~ //Centered across the screen
	 //~ fieldX1 = centerX - width/2;
    //~ fieldX2 = centerX + width/2;
    //~ fieldY1 = centerY - height/2;
    //~ fieldY2 = centerY + height/2;
	 //~ //-----------------------------
	 
	 cout << "Device Coordinates(" << fieldX1 << ", " << fieldY1 << ", " << fieldX2 << ", " << fieldY2 << ")\n";
	 cout << "Ratio: " << float(fieldX2-fieldX1)/float(fieldY2-fieldY1) << endl;
//--------------------------------------------------------	
	
	
	
	
	float targetRatio = WORLD_MAXX/WORLD_MAXY;
	float ratio;
	float numerator;
	float denominator;
	
	numerator = float(fieldX2)-float(fieldX1);
	denominator = float(fieldY2)-float(fieldY1);
	
	ratio = numerator/denominator;
	
	
	//////////////////////////////////////////////////////////////////
	
	cout << "Device Coordinates(" << fieldX1 << ", " << fieldY1 << ", " << fieldX2 << ", " << fieldY2 << ")\n";
	cout << "(centerX, centerY) = " << "(" << centerX << ", " << centerY << ")" << endl;
	cout << "Target Ratio: " << targetRatio << endl;
	cout << "Ratio: " << float(fieldX2-fieldX1)/float(fieldY2-fieldY1) << endl;

	//////////////////////////////////////////////////////////////////
		    
//5 vs 5 MIROSOT FIELD    
	//World boundaries
    worldBoundary.x1 = 0.0;
    worldBoundary.y1 = WORLD_MAXY;
    worldBoundary.x2 = WORLD_MAXX;
    worldBoundary.y2 = 0.0;

	//Device boundaries
    deviceBoundary.x1 = fieldX1;
    deviceBoundary.y1 = fieldY1;
    deviceBoundary.x2 = fieldX2;
    deviceBoundary.y2 = fieldY2;
	 

	 robotWidth = xDev(worldBoundary,deviceBoundary,7.5f);//-deviceBoundary.x1;
	 //robotWidth = xDev(worldBoundary,deviceBoundary,8.0f)-deviceBoundary.x1;
	 cout << "0.5 cm = " << halfCentimeter << endl;
	 
	 
	 
	 cout << "calculated 7.5cm = " << robotWidth << endl;
	 robotWidth = oneCentimeter * 7 + halfCentimeter;
	 cout << "7.5cm = " << robotWidth << endl;
	 xInc = robotWidth;
	 yInc = robotWidth;
}


void drawGridWorld(char* title, int xInc, int yInc, int &rows, int &cols){
	int countVertLines, countHorizLines;
	
	countVertLines=0;
	countHorizLines=0;
	
	rows=0;
	cols=0;
	
	
	setbkcolor(BLACK);
	settextjustify(CENTER_TEXT, CENTER_TEXT);
	//outtextxy(getmaxx()/2, textheight("H")/2, title);
	//outtextxy(getmaxx()/2, textheight("H"), title);
	outtextxy(getmaxx()/2, 0, title);
	
   setlinestyle(WIDE_DOT_FILL, 1, 1);
	
   
	setcolor(LINE_COLOUR);
   //draw vertical bars
	
	
	for(int x=deviceBoundary.x1; x <= deviceBoundary.x2; x += xInc){
		line(x, fieldY1, x, fieldY2);
		cols++;
	}
	
	for(int y=deviceBoundary.y1; y <= deviceBoundary.y2; y += yInc){
		line(fieldX1, y, fieldX2, y);
		rows++;
	}
   
	setcolor(LINE_COLOUR);
	rectangle(fieldX1, fieldY1, fieldX2, fieldY2);
	
	//~ cout << "Total rows = " << rows << endl;
	//~ cout << "Total cols = " << cols << endl;
	
}


void loadAndDisplayMap(const char* fn, vertex map_d[GRIDWORLD_ROWS][GRIDWORLD_COLS], int totalRows, int totalColumns){
	ifstream i_file;
	
	i_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
	
	try{
		i_file.open(fn, std::ifstream::in);
			
			for(int j =0; j<GRIDWORLD_ROWS; j++) //row
			{
				for(int i =0;i<GRIDWORLD_COLS;i++) //col
				{
					i_file >> map_d[j][i].type;
					
					if(map_d[j][i].type == '0') //traversable cell
					{
						drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, NULL);
					}
					
					if(map_d[j][i].type == '1') //'B' - blocked cell
					{
						drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, NULL);
					}
					if(map_d[j][i].type == '9') //unknown
					{
						drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, NULL);
					}
					if(map_d[j][i].type == '6') //'S' - start vertex
					{
						drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, "S");
					}
					if(map_d[j][i].type == '7') //'G' - goal vertex
					{
						drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, "G");
					}
				}
			}
		
		i_file.close();
	}
	catch (std::ifstream::failure e) {
      std::cerr << "Exception opening/reading/closing file\n";
  }
  //cout << "\nGridworld loaded from file" << endl;


}

void saveMap(const char* fn){
	
	std::ofstream o_file;
  // o_file.open (fn);
	
 
	
	o_file.exceptions ( std::ofstream::failbit | std::ifstream::badbit );
	
	try{
		o_file.open(fn, std::ofstream::out);
		if (o_file.is_open())
      {	
			//------------------------------------------------------
			o_file << "===== g-values =====" << endl;
			for(int j =0; j<GRIDWORLD_ROWS; j++) //row
			{
				for(int i =0;i<GRIDWORLD_COLS;i++) //col
				{
					o_file.width(4);
					o_file.precision(3);
					o_file << map[j][i].g << ", ";
					
					//o_file << 1;
				}
				o_file << endl << endl;
			}
			o_file << "====================" << endl;
			//------------------------------------------------------
			// h-values
			//
			o_file << "===== h-values =====" << endl;
			for(int j =0; j<GRIDWORLD_ROWS; j++) //row
			{
				for(int i =0;i<GRIDWORLD_COLS;i++) //col
				{
					o_file.width(4);
					o_file.precision(3);
					o_file << map[j][i].h << ", ";
					
					//o_file << 1;
				}
				o_file << endl << endl;
			}
			o_file << "====================" << endl;
			//------------------------------------------------------
			
			// f-costs
			//
			o_file << "===== f-costs =====" << endl;
			for(int j =0; j<GRIDWORLD_ROWS; j++) //row
			{
				for(int i =0;i<GRIDWORLD_COLS;i++) //col
				{
					o_file.width(4);
					o_file.precision(3);
					o_file << map[j][i].f << ", ";
					
					//o_file << 1;
				}
				o_file << endl << endl;
			}
			o_file << "====================" << endl;
		
		   o_file.close();
		}
		else
		  {
			 std::cout << "Error opening file";
		  }
	}
	catch (std::ofstream::failure e) {
    std::cerr << "Exception opening/writing/closing file\n";
  }
  cout << "\nGridworld values saved into file.\n" << endl << "--------" << endl;


}


void createDummyMap(){
	for(int j =0; j<GRIDWORLD_ROWS; j++)
		{
			for(int i =0;i<GRIDWORLD_COLS;i++)
			{
			
				
				if(map[j][i].type == '0') //traversable cell
				{
					map[j][i].g = randomVal(1, 20);
				}
				
				if(map[j][i].type == '1') //'B' - blocked cell
					map[j][i].g = 99;
				if(map[j][i].type == '9') //unknown
					map[j][i].g = 99;
				if(map[j][i].type == '6') //'S' - start
					map[j][i].g = 0;
				if(map[j][i].type == '7') //'G' - goal
					map[j][i].g = 99;
				
			}
		}
}

void resetMap(){
	
	for(int j =0; j<GRIDWORLD_ROWS; j++)
	{
			for(int i =0;i<GRIDWORLD_COLS;i++)
			{
				map[j][i].type = BLOCKED;
				map[j][i].g = 0;
				map[j][i].h = 0;
				map[j][i].rhs = 0;
				map[j][i].f = (double)0.0;
			}
	}
	
}

void restart(){
	for(int j =0; j<GRIDWORLD_ROWS; j++){
		for(int i =0;i<GRIDWORLD_COLS;i++){
			map[j][i].status = UNEXPLORED;
			//map[j][i].g = 0;
			//map[j][i].h = 0;
			//map[j][i].rhs = 0;
			//map[j][i].f = (double)0.0;
		}
	}
}

void displayGValuesMap(vector<CellPosition> p,int totalRows, int totalColumns){
	
	char tmp[4];
	int numCharsRead=0;
	
		
		for(int j=0; j<GRIDWORLD_ROWS; j++)
		{
			for(int i =0;i<GRIDWORLD_COLS;i++)
			{
				
				strcpy(tmp,"");
				numCharsRead=sprintf(tmp,"%3.2f",map[j][i].g); 
				if(map[j][i].type == '0') //traversable cell
				{
					
					
					
						if(numCharsRead){
							if(map[j][i].g > 0.0){
								
								if(map[j][i].status == EXPANDED){
								    drawCell_RowCol(MAGENTA, j,i, totalRows, totalColumns, tmp); //expanded cell
								} else {
									drawCell_RowCol(LIGHTMAGENTA, j,i, totalRows, totalColumns, tmp);
								}
								
								
							} else {
								drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, tmp);
							}
						} else {
							drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, NULL);
						}
				   
				}
				if(map[j][i].type == '1') //'B' - blocked cell
				{
					
						if(numCharsRead){ 					
							drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, tmp);
					   } else {
						   drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, NULL);
					   }
				}
					
				if(map[j][i].type == '9') //unknown
				{
					
					if(numCharsRead) 	
					{						
						drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, tmp);
					} else {
						drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, NULL);
					}
				}	
					
				if(map[j][i].type == '6') //'S' - start
				{		
                  if(numCharsRead) 	
						{							
							drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, tmp);
					   } else {
						   drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, NULL);
					   }
				}		
				
					
				if(map[j][i].type == '7') //'G' - goal
				{
					if(numCharsRead) 		
					{						
							drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, tmp);
					} else {
						   drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, NULL);
					}
					
				}			
					
			}
		}
	
	//////////////////////////////////////////////
	// showPath
	int length = p.size();
   	
		
   for(int i=0; i < length; i++){
		markWayPoint_RowCol(RED, p[i].row,p[i].col, totalRows, totalColumns);
		strcpy(tmp,"");
		numCharsRead=sprintf(tmp,"%3.2f",map[p[i].row][p[i].col].g); 
		if(numCharsRead){ 					
			drawPathCell_RowCol(YELLOW, p[i].row,p[i].col, totalRows, totalColumns, tmp);
		}
		
	}		

}
void displayFValuesMap(vector<CellPosition> p,int totalRows, int totalColumns){
	
	char tmp[4];
	int numCharsRead=0;
	
		
		for(int j=0; j<GRIDWORLD_ROWS; j++)
		{
			for(int i =0;i<GRIDWORLD_COLS;i++)
			{
				
				strcpy(tmp,"");
				numCharsRead=sprintf(tmp,"%3.2f",map[j][i].f); 
				if(map[j][i].type == '0') //traversable cell
				{
					
					if(numCharsRead){ 					
						//drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, tmp);
						if(map[j][i].f > 0.0){
						   drawCell_RowCol(CYAN, j,i, totalRows, totalColumns, tmp);
						} else {
						   drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, tmp);
						}
						
						
						
					} else {
						drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, NULL);
					}
				}
				if(map[j][i].type == '1') //'B' - blocked cell
				{
					
						if(numCharsRead){ 					
							drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, tmp);
					   } else {
						   drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, NULL);
					   }
				}
					
				if(map[j][i].type == '9') //unknown
				{
					
					if(numCharsRead) 	
					{						
						drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, tmp);
					} else {
						drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, NULL);
					}
				}	
					
				if(map[j][i].type == '6') //'S' - start
				{		
                  if(numCharsRead) 	
						{							
							drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, tmp);
					   } else {
						   drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, NULL);
					   }
				}		
				
					
				if(map[j][i].type == '7') //'G' - goal
				{
					if(numCharsRead) 		
					{						
							drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, tmp);
					} else {
						   drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, NULL);
					}
					
				}			
					
			}
		}
	
	//////////////////////////////////////////////
	// showPath
	int length = p.size();
   	
		
   for(int i=0; i < length; i++){
		markWayPoint_RowCol(RED, p[i].row,p[i].col, totalRows, totalColumns);
		strcpy(tmp,"");
		numCharsRead=sprintf(tmp,"%3.2f",map[p[i].row][p[i].col].f); 
		if(numCharsRead){ 					
			drawPathCell_RowCol(YELLOW, p[i].row,p[i].col, totalRows, totalColumns, tmp);
		}
		
	}		

}

void displayRhsValuesMap(vector<CellPosition> p,int totalRows, int totalColumns){
	
	char tmp[4];
	int numCharsRead=0;
	
		
		for(int j=0; j<GRIDWORLD_ROWS; j++)
		{
			for(int i =0;i<GRIDWORLD_COLS;i++)
			{
				
				strcpy(tmp,"");
				numCharsRead=sprintf(tmp,"%3.2d",map[j][i].rhs); 
				if(map[j][i].type == '0') //traversable cell
				{
					
					if(numCharsRead){ 					
						//drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, tmp);
						if(map[j][i].rhs > 0.0){
						   drawCell_RowCol(MAGENTA, j,i, totalRows, totalColumns, tmp);
						} else {
						   drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, tmp);
						}
						
						
					} else {
						drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, NULL);
					}
				}
				if(map[j][i].type == '1') //'B' - blocked cell
				{
					
						if(numCharsRead){ 					
							drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, tmp);
					   } else {
						   drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, NULL);
					   }
				}
					
				if(map[j][i].type == '9') //unknown
				{
					
					if(numCharsRead) 	
					{						
						drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, tmp);
					} else {
						drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, NULL);
					}
				}	
					
				if(map[j][i].type == '6') //'S' - start
				{		
                  if(numCharsRead) 	
						{							
							drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, tmp);
					   } else {
						   drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, NULL);
					   }
				}		
				
					
				if(map[j][i].type == '7') //'G' - goal
				{
					if(numCharsRead) 		
					{						
							drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, tmp);
					} else {
						   drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, NULL);
					}
					
				}			
					
			}
		}
	
	//////////////////////////////////////////////
	// showPath
	int length = p.size();
   	
		
   for(int i=0; i < length; i++){
		markWayPoint_RowCol(RED, p[i].row,p[i].col, totalRows, totalColumns);
		strcpy(tmp,"");
		numCharsRead=sprintf(tmp,"%3.2d",map[p[i].row][p[i].col].rhs); 
		if(numCharsRead){ 					
			drawPathCell_RowCol(YELLOW, p[i].row,p[i].col, totalRows, totalColumns, tmp);
		}
		
	}		

}

void displayHValuesMap(vector<CellPosition> p,int totalRows, int totalColumns){
	
	char tmp[4];
	int numCharsRead=0;
	
		
		for(int j=0; j<GRIDWORLD_ROWS; j++)
		{
			for(int i =0;i<GRIDWORLD_COLS;i++)
			{
				
				strcpy(tmp,"");
				numCharsRead=sprintf(tmp,"%3.2f",map[j][i].h); 
				if(map[j][i].type == '0') //traversable cell
				{
					
					if(numCharsRead){ 					
						//drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, tmp);
						if(map[j][i].h > 0.0){
						   drawCell_RowCol(LIGHTRED, j,i, totalRows, totalColumns, tmp);
						} else {
						   drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, tmp);
						}
					} else {
						drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, NULL);
					}
				}
				if(map[j][i].type == '1') //'B' - blocked cell
				{
					
						if(numCharsRead){ 					
							drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, tmp);
					   } else {
						   drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, NULL);
					   }
				}
					
				if(map[j][i].type == '9') //unknown
				{
					
					if(numCharsRead) 	
					{						
						drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, tmp);
					} else {
						drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, NULL);
					}
				}	
					
				if(map[j][i].type == '6') //'S' - start
				{		
                  if(numCharsRead) 	
						{							
							drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, tmp);
					   } else {
						   drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, NULL);
					   }
				}		
				
					
				if(map[j][i].type == '7') //'G' - goal
				{
					if(numCharsRead) 		
					{						
							drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, tmp);
					} else {
						   drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, NULL);
					}
					
				}			
					
			}
		}
	int length = p.size();
   	
		
   for(int i=0; i < length; i++){
		markWayPoint_RowCol(RED, p[i].row,p[i].col, totalRows, totalColumns);
		strcpy(tmp,"");
		numCharsRead=sprintf(tmp,"%3.2f",map[p[i].row][p[i].col].h); 
		if(numCharsRead){ 					
			drawPathCell_RowCol(YELLOW, p[i].row,p[i].col, totalRows, totalColumns, tmp);
		}
		
	}		

}





void drawGrid(int xInc, int yInc){
	int countVertLines, countHorizLines;
	
	countVertLines=0;
	countHorizLines=0;
	
   setlinestyle(WIDE_DOT_FILL, 1, 1);   
	setcolor(LINE_COLOUR);

   for(int i=fieldX1; i < fieldX2; i += xInc){
	   line(i,fieldY1 ,i,fieldY2);
		countVertLines++;
	}
   
   for(int i=fieldY1; i < fieldY2; i += yInc){
	   line(fieldX1,i,fieldX2,i);
		countHorizLines++;
	}
	
	setcolor(WHITE);
	rectangle(fieldX1, fieldY1, fieldX2, fieldY2);
	
}

void drawInformationPanel(int x, int y, char* info){
	settextstyle(SMALL_FONT, HORIZ_DIR, 6);
	settextjustify(LEFT_TEXT,CENTER_TEXT);
	setcolor(YELLOW);
	outtextxy(x ,y, info);
}

void markCell_XY(int color_c, int x, int y, int rows, int cols){
	
	int cellX, cellY;
	int selectedCol;
	int selectedRow;
	
	cellX = deviceBoundary.x1 + (((x-deviceBoundary.x1)/robotWidth) * robotWidth);
	cellY = deviceBoundary.y1 + (((y-deviceBoundary.y1)/robotWidth) * robotWidth);
	
	selectedCol = (cellX-deviceBoundary.x1)/ robotWidth;
	selectedRow = (cellY-deviceBoundary.y1)/ robotWidth;
	
	if(selectedCol == (cols-1)){
		cellX = cellX - robotWidth;
		--selectedCol ;
	}
	
	setcolor(color_c);
	rectangle(cellX, cellY, cellX+robotWidth, cellY+robotWidth);
	
	cout << "selected (Row, Col) = (" << selectedRow << ", " << selectedCol << ")" << endl;
	
}


void markWayPoint_XY(int color_d,int x, int y, int rows, int cols, char* text){
	
	int cellX, cellY;
	int selectedCol;
	int selectedRow;

	
	cellX = deviceBoundary.x1 + (((x-deviceBoundary.x1)/robotWidth) * robotWidth);
	cellY = deviceBoundary.y1 + (((y-deviceBoundary.y1)/robotWidth) * robotWidth);
	
	selectedCol = (cellX-deviceBoundary.x1)/ robotWidth;
	selectedRow = (cellY-deviceBoundary.y1)/ robotWidth;
	
	if(selectedCol == (cols-1)){
		cellX = cellX - robotWidth;
		--selectedCol ;
	}
	setcolor(color_d);
	rectangle(cellX, cellY, cellX+robotWidth, cellY+robotWidth);
	setcolor(color_d);
	
	settextstyle(SMALL_FONT, HORIZ_DIR,7);
	settextjustify(CENTER_TEXT, CENTER_TEXT);
	setbkcolor(color_d);
	outtextxy(cellX+(robotWidth/2), cellY+(robotWidth/2)+(textheight("S")/2),text);
	
	setbkcolor(BLACK);
	
	char info[80]; 
	sprintf(info,"row: %d, col: %d", selectedRow, selectedCol); 
			  
	drawInformationPanel(fieldX2+(textwidth("XX")),fieldY1+(4*textheight("X")), info);
	
}


void drawPathCell_RowCol(int color_d,int row, int col, int totalRows, int totalColumns, char* text){
	int cellX, cellY;
	
	cellX = deviceBoundary.x1 + (col * robotWidth);
	cellY = deviceBoundary.y1 + (row * robotWidth);
	
	if(col == (totalColumns-1)){
		cellX = cellX - robotWidth;
		--col ;
	}
	
	setbkcolor(BLACK);
	setcolor(color_d);
	rectangle(cellX, cellY, cellX+robotWidth, cellY+robotWidth);
	setfillstyle(SOLID_FILL,color_d);
	fillellipse(cellX+(robotWidth/2), cellY+(robotWidth/2), robotWidth/2, robotWidth/2);
	setcolor(LINE_COLOUR);
	
	if(text != NULL){
		settextstyle(SMALL_FONT, HORIZ_DIR,4);
		settextjustify(CENTER_TEXT, CENTER_TEXT);
		setbkcolor(color_d);
	   setcolor(RED);
		outtextxy(cellX+(robotWidth/2), cellY+(robotWidth/2)+(textheight("S")/2),text);
		
	}
	setbkcolor(BLACK);
}


void drawCell_RowCol(int color_d,int row, int col, int totalRows, int totalColumns, char* text){
	int cellX, cellY;
	
	cellX = deviceBoundary.x1 + (col * robotWidth);
	cellY = deviceBoundary.y1 + (row * robotWidth);
	
	if(col == (totalColumns-1)){
		cellX = cellX - robotWidth;
		--col ;
	}
	
	setbkcolor(BLACK);
	setcolor(color_d);
	rectangle(cellX, cellY, cellX+robotWidth, cellY+robotWidth);
	//setfillstyle(SOLID_FILL,WHITE);
	setfillstyle(SOLID_FILL,color_d);
	bar(cellX, cellY, cellX+robotWidth, cellY+robotWidth);
	setcolor(LINE_COLOUR);
	rectangle(cellX, cellY, cellX+robotWidth, cellY+robotWidth);
	
	if(text != NULL){
		//settextstyle(SMALL_FONT, HORIZ_DIR,7);
		settextstyle(SMALL_FONT, HORIZ_DIR,4);
		settextjustify(CENTER_TEXT, CENTER_TEXT);
		setbkcolor(color_d);
		
		//----------------------------
		//~ if(color == LIGHTGRAY){
			//~ setcolor(BLACK);
		//~ } else if(color == BLUE){
		   //~ setcolor(YELLOW);
		//~ } else {
			//~ setcolor(RED);
		//~ }
	   //setcolor(RED);
		setcolor(DARKGRAY);
		outtextxy(cellX+(robotWidth/2), cellY+(robotWidth/2)+(textheight("S")/2),text);
		
	}
	setbkcolor(BLACK);
}

void markWayPoint_RowCol(int color_d,int row, int col, int totalRows, int totalColumns){
	
	int cellX, cellY;
	
	cellX = deviceBoundary.x1 + (col * robotWidth);
	cellY = deviceBoundary.y1 + (row * robotWidth);
	
	
	if(col == (totalColumns-1)){
		cellX = cellX - robotWidth;
		--col ;
	}
	//circle(cellX, cellY, 50);
	setcolor(WHITE);
	rectangle(cellX, cellY, cellX+robotWidth, cellY+robotWidth);
	setfillstyle(SOLID_FILL, WHITE);
	fillellipse(cellX+(robotWidth/2), cellY+(robotWidth/2), robotWidth/3, robotWidth/3);
	setcolor(color_d);
	circle(cellX+(robotWidth/2), cellY+(robotWidth/2), robotWidth/3);
	
}

void displayPath(vector<CellPosition> p, int totalRows, int totalColumns){
	int length = p.size();
	for(int i=0; i < length; i++){
		markWayPoint_RowCol(RED, p[i].row,p[i].col, totalRows, totalColumns);
	}
}



void initGridWorld(string fn){
	ifstream i_file;
	
	i_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
	
	try{
		i_file.open(fn.c_str(), std::ifstream::in);
			
			for(int j =0; j<GRIDWORLD_ROWS; j++){
				for(int i =0;i<GRIDWORLD_COLS;i++){
					i_file >> map[j][i].type;		
					map[j][i].status = UNEXPLORED;
					map[j][i].row = j;
					map[j][i].col = i;
					if(map[j][i].type == '6'){
						startVertex = &map[j][i];
						startVertex->steps = 0;
					}
					else if(map[j][i].type == '7'){
						goalVertex  = &map[j][i];
					}
				}
			}
		
		i_file.close();
	}
	catch (std::ifstream::failure e) {
      std::cerr << "Exception opening/reading/closing file\n";
  }
}

void changeGridWorld(string fn){
	
	ifstream i_file;	
	i_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );	
	bool error = false;
	try{
		i_file.open(fn.c_str(), std::ifstream::in);
			for(int j =0; j<GRIDWORLD_ROWS && !error; j++){
				for(int i =0;i<GRIDWORLD_COLS && !error;i++){
					i_file >> map2[j][i].type;				
					/*map2[j][i].rhs = MAX_INT;
					map2[j][i].g = 0.0;
					map2[j][i].c = MAX_INT;
					map2[j][i].h = 0.0;
					map2[j][i].f = 0.0;*/
					if(map2[j][i].type == '6' || map[j][i].type == '6'){
						if(map[j][i].type != '6' || map2[j][i].type != '6'){
							cerr << "Error: cannot change the start location with an update." << endl;
							error = true;
						}
					}
					else if(map2[j][i].type == '7' || map[j][i].type == '7'){
						if(map[j][i].type != '7' || map2[j][i].type != '7'){
							cerr << "Error: cannot change the goal location with an update." << endl;
							error = true;
						}
					}
				}
			}
		
		i_file.close();
	}
	catch (std::ifstream::failure e) {
      std::cerr << "Exception opening/reading/closing file\n";
   }
   if(error)
		initGridWorld(initGridworldFileName);
}

void runSimulation(){
	
	clock_t  startTime;
	float elapsedTime = 0;
	
	bool change_occured = false;
	vector<CellPosition> path;
	char algorithmName[80];
	bool ANIMATE_MOUSE_FLAG=false;
	bool validCellSelected=false;
	static BOOL page=false;
	int mX = 0, mY = 0;
	float worldX, worldY;
	worldX=0.0f;
	worldY=0.0f;
	int xInc=0;
	int yInc=0;
	int mouseRadius=1;
	int rows, cols;
	
	rows = 0;
	cols = 0;
	
	strcpy(algorithmName,"Test Algorithm");
	srand(time(NULL));  // Seed the random number generator
			
	//Initialise the world boundaries
   initSystemOfCoordinates(xInc, yInc);
	
	initGridworldFileName="init.txt";
	outputGridworldFileName="output.txt";
	
	resetMap();
	drawGridWorld(algorithmName,xInc, yInc, rows, cols);
	loadAndDisplayMap(initGridworldFileName.c_str(), map, rows, cols);
	initGridWorld(initGridworldFileName);
	
	while(1){ //main loop
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	char answer = 'N';
	//string initialState, goalState;
		
	
	//path.clear();
   
   do{
		cout << "===============================================================" << endl;
		cout << "               << 159741 - Intelligent Robotics >>" << endl;
	   cout << endl << "                    Path-Planning Algorithms" << endl;
	   cout << "===============================================================" << endl;
	   cout << "(1) Breadth First Search (with Visited List)" << endl << "(2) A* (without Expanded List)" << endl << "(3) A* (using the Strict Expanded List)" << endl << "(4) LifeLong Planning A*" << endl << "(5) D* Lite" 
		     << endl << "(I) Initial State" << endl << "(C) Changed State" << endl << "(G) Display map of the g, h and f-values" << endl << "(H) Display map of the h-values" << endl << "(F) Display map of the f-values" << endl << "(R) Display map of the rhs-values" << endl << "(S) Save map of g-values" << endl << "(Q) Quit" << endl << endl;  

	   cout << "Type selection: ";
	   cin >> answer;
		cout << endl;
		
	   
	   if(answer == 'Q' || answer == 'q') exit(0);
	   
	   
	   
	   switch(toupper(answer)){
		   case 'I':  
						  cout << "Enter file name of initial gridworld: " << endl;
                    cin >> initGridworldFileName;
			           resetMap();
						  initGridWorld(initGridworldFileName);
						  change_occured = false;
			           path.clear();
			           break; 
			
			case 'C':  
						  cout << "Enter file name of changed gridworld: " << endl;
                    cin >> changedGridworldFileName;
						  changeGridWorld(changedGridworldFileName);
			           path.clear();
						  change_occured = true;
			           break; 
			case 'S':
				        saveMap(outputGridworldFileName.c_str());  
                    break;				
		   
			case 'G':  cout << "Display map of the g-values" << endl;
			           displayGValuesMap(path, rows,cols);
			           cout << "click on the graphics window, then press any key to continue... " << endl;
			           getch();
		              break;
			case 'H':  cout << "Display map of the h-values" << endl;
			           displayHValuesMap(path, rows,cols);
			           cout << "click on the graphics window, then press any key to continue... " << endl;
			           getch();
		              break; 
			case 'F':  cout << "Display map of the f-costs" << endl;
			           displayFValuesMap(path, rows,cols);
			           cout << "click on the graphics window, then press any key to continue... " << endl;
			           getch();
		              break;
			
			case 'R':  cout << "Display map of the rhs-values" << endl;
			           displayRhsValuesMap(path, rows,cols);
			           cout << "click on the graphics window, then press any key to continue... " << endl;
			           getch();
		              break;
           
		   case '1':  
                 	  cout << "Breadth First Search" << endl << endl;
                    strcpy(algorithmName,"Breadth_First_Search");
			
			           startTime = clock();
						  
			           createDummyMap();  //THIS IS FOR TESTING ONLY.. DISABLE THIS FUNCTION CALL ONCE YOU HAVE IMPLEMENTED THE ALGORITHMS
			           path = testAlgorithm();
			
			           elapsedTime = (float)(clock() - startTime) / CLOCKS_PER_SEC;

		   
		              break;
			case '2':  cout << "A* (without Expanded List) " << endl;
						  strcpy(algorithmName,"AStar");
			           
						  break;
			
			case '3':
						  cout << "A* (using the Strict Expanded List)" << endl;
			           strcpy(algorithmName,"AStar Strict Expanded List");
			           startTime = clock();
						  restart();
			           path = a_star(startVertex, goalVertex); //implement this function	
			           elapsedTime = (float)(clock() - startTime) / CLOCKS_PER_SEC;
						  break;
			
			case '4':
						  cout << "Lifelong Planning A*" << endl;
						  if(strcmp(algorithmName, "LPAstar")){
							  cout << "~*~*~*~*~restarting~*~*~*~*~*~" << endl;
							  restart();
						  }
			           startTime = clock();
			           strcpy(algorithmName,"LPAstar");
						  path = LPAStar(startVertex, goalVertex, change_occured);  //implement this function
			           elapsedTime = (float)(clock() - startTime) / CLOCKS_PER_SEC;
						  break;
			case '5':
						  cout << "D* Lite" << endl;
			           strcpy(algorithmName,"D* Lite");
						  //path = dStarLite(startVertex, goalVertex);  //implement this function
						  break;
		}
		   
	} while(answer == 'N');

	
	if(strcmp(algorithmName,"")){
		cout << "--------------<< RESULTS >>---------------" << endl << endl;
		cout << "Selected Algorithm: " << algorithmName <<  endl;
		
		cout << "ElapsedTime = " << elapsedTime << " sec." << endl;
		cout << "numberOfExpandedStates = " << numberOfExpandedStates << " states" << endl;
		cout << "------------------------------------------" << endl << endl;
		
		
		
   }
	
	
	cout << "press Escape first to select another algorithm." << endl;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// keep running the program until the ESC key is pressed   
	while((GetAsyncKeyState(VK_ESCAPE)) == 0 ) {
			 setactivepage(page);
			 cleardevice();

		    drawGridWorld(algorithmName,xInc, yInc, rows, cols);
		    
		    loadAndDisplayMap(initGridworldFileName.c_str(), map,rows,cols);
		
		 
		    if(path.size() > 0) displayPath(path, rows, cols);
			 
		
	   //----------------------------------------------------------------------------------------------------------------	  
		// Mouse handling
		//
			 if(mousedown()){
						 				
				ANIMATE_MOUSE_FLAG=true;
				 			 
				mX = mousecurrentx();
				mY = mousecurrenty();
				 
				 //if the goal selected is within the playing field boundaries
  	         if(mX >= fieldX1 && mX <= fieldX2 && mY >= fieldY1 && mY <= fieldY2){
					
					    circle(mX, mY, 3);
					    validCellSelected = true;
  	       
				} else {
					validCellSelected = false;
				}
			 } //end of mousedown()
			 //------------------------------------------------------------------------------------------------------------------
			 /////////////////////////////////////////////////////////////////////////////
			 
						 
			 if(ANIMATE_MOUSE_FLAG){
				 
				  //draw Cross-hair to mark Goal	    
				  setcolor(RED);
				  circle(mX, mY, 20);
				  line(mX,mY-20,mX,mY+20);
				  line(mX-20,mY,mX+20,mY);
				  //end of draw Cross-hair 
			 
				  // special effect to display concentric circles locating the target
					setcolor(YELLOW);
					
					if(mouseRadius < 110) {
						mouseRadius += 1;
					}
					circle(mX, mY, mouseRadius);
														
					if(mouseRadius >= 110) {
						ANIMATE_MOUSE_FLAG=false;
						mouseRadius=0;
					}
					//end of special effect
			  }
						 					 
			 
			 /////////////////////////////////////////////////////////////////////////////
			  char info[80]; 
			  sprintf(info,"x: %d, y: %d",mX, mY); 
			  //drawInformationPanel(fieldX1+textwidth("XXX"),fieldY1-(textheight("X")), info);
			  drawInformationPanel(fieldX2+(2*textwidth("XX")),fieldY1+(textheight("X")), info);
			  
			 
			  /////////////////////////////////////////////////////////////////////////////
			  //for debugging only
			  #ifdef SHOW_DEBUG_INFO
			  float wX, wY;
			  
			  wX = xWorld(worldBoundary,deviceBoundary,mX);
			  wY = yWorld(worldBoundary,deviceBoundary,mY);
			  sprintf(info,"worldX: %f, worldY: %f",wX, wY); 
			  //drawInformationPanel(fieldX1+textwidth("XXX"),fieldY1-(2*textheight("X")), info);
			  drawInformationPanel(fieldX2+textwidth("XXX"),fieldY1+(2*textheight("X")), info);
			  
			  #endif
			 ///////////////////////////////////////////////////////////////////////////// 
			  
			  
			  if(validCellSelected) {
				  
				  int mButton = whichmousebutton();

				  switch(mButton){	
					  case LEFT_BUTTON:
									markWayPoint_XY(BLUE, mX, mY, rows, cols, "S");	  
						         break;
					  
					  case RIGHT_BUTTON:
									markWayPoint_XY(GREEN, mX, mY, rows, cols, "G");	  
						         break;
				  
				  };
				  
			  }
			  setvisualpage(page);
			  page = !page;  //switch to another page
	}
	
	
 } //main loop
 
}


int main(void) {
	
	int graphDriver = 0,graphMode = 0;
 	
 	initgraph(&graphDriver, &graphMode, "", 1280, 800); // Start Window	
	//initgraph(&graphDriver, &graphMode, "", 1920, 1080); // Start Window - Full-HD	
	
	BACKGROUND_COLOUR = WHITE;
	LINE_COLOUR = GREEN;
	
	#ifdef EIGHT_CONNECTED_GRIDWORLD
	MAX_MOVES = 8;
	#endif
	
	#ifdef FOUR_CONNECTED_GRIDWORLD
	MAX_MOVES = 4;
	#endif	

   try{
		runSimulation();
   }

   catch(...){
    	cout << "Exception caught!\n";
   }
	cout << "----<< The End.>>----" << endl;
	
	return 0;
} 









