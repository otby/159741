#include "searchUtility.h"

double heuristic(vertex *s, vertex *g){
	return sqrt((s->row-g->row)*(s->row-g->row)+(s->col-g->col)*(s->col-g->col));
}


