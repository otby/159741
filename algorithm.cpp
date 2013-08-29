#include <algorithm>
#include <iostream>
#include <vector>

#include "algorithm.h"
#include "graphics.h"

using namespace std;


//----------------------------------------------------------------------------------------


vector<CellPosition> testAlgorithm(){
	
	vector<CellPosition>  path;
	CellPosition c;
	
	c.row = 11;
	c.col = 6;
	path.push_back(c);
	
	c.row = 11;
	c.col = 7;
	path.push_back(c);
	
	c.row = 11;
	c.col = 8;
	path.push_back(c);
	
	c.row = 10;
	c.col = 9;
	path.push_back(c);
	
	c.row = 10;
	c.col = 10;
	path.push_back(c);
	
	c.row = 10;
	c.col = 11;
	path.push_back(c);
	
	c.row = 10;
	c.col = 12;
	path.push_back(c);
	
	c.row = 10;
	c.col = 13;
	path.push_back(c);
	
	c.row = 11;
	c.col = 14;
	path.push_back(c);

   c.row = 11; 
	c.col = 15;
	path.push_back(c);
   c.row = 11; 
	c.col = 16;
	path.push_back(c);
   c.row = 11; 
	c.col = 17;
	path.push_back(c);


		
	return path;
}


//--------------------------------------------------------------------------------------------------











