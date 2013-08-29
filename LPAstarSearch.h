#ifndef __LPASTARSEARCH_H__
#define __LPASTARSEARCH_H__

#include <iostream>
#include <stdio.h>
#include <queue>
#include <list>
#include <vector>
#include <cmath>
#include <algorithm>
#include "globalVariables.h"
#include "searchUtility.h"

bool lpa_compare (vertex* lhs, vertex* rhs);

extern vector<CellPosition> lpa_path;
//extern priority_queue<vertex*,vector<vertex*>,lpa_compare> lpa_q;
extern list<vertex*> lpa_q;

void lpa_update(vertex* v);
double lpa_heuristic(vertex *s, vertex *g);
vertex* lpa_succ(vertex* v, int i);
vector<CellPosition> LPAStar(vertex* start, vertex* goal, bool change);
void lpa_change();
bool check_path();

#endif
