#ifndef __ASTARSEARCH_H__
#define __ASTARSEARCH_H__

#include <iostream>
#include <stdio.h>
#include <queue>
#include <vector>
#include <cmath>
#include <malloc.h>
#include <assert.h>
#include "globalVariables.h"
#include "searchUtility.h"


struct astar_compare
{
    bool operator() (vertex* lhs, vertex* rhs) { return lhs->f >= rhs->f; }
};

extern vector<CellPosition> astar_path;
extern priority_queue<vertex*,vector<vertex*>,astar_compare> astar_q;

vertex* astar_succ(vertex* v, short i);
double astar_heuristic(vertex *s, vertex *g);
vector<CellPosition> a_star(vertex* s, vertex* g);

#endif
