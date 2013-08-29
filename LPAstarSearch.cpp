#include "LPAstarSearch.h"

vector<CellPosition> lpa_path;
list<vertex*> lpa_q;

double lpa_heuristic(vertex *s, vertex *g){
	return sqrt((s->row-g->row)*(s->row-g->row)+(s->col-g->col)*(s->col-g->col));
}

bool lpa_compare (vertex* lhs, vertex* rhs) { 
	//cout << "comparing: [" << lhs->key1 << ", " << lhs->key2 << "] and [" << rhs->key1 << ", " << rhs->key2 << "]" << endl;
	if(lhs->key1 > rhs->key1){
		return false;
	}else if (lhs->key1 < rhs->key1){
		return true;
	}else if(lhs->key2 >= rhs->key2) {
		return false;
	} else {
		return true;
	}
}

vertex* lpa_edge(vertex* v, int i){
	int x, y;
	switch (i){
		case 1:
			x = 1; y = -1;
			break;
		case 2:
			x = 1; y = 0;	
			break;		
		case 3:
			x = 1; y = 1;
			break;
		case 4:
			x = 0; y = 1;
			break;
		case 5:
			x = -1; y = 1;
			break;
		case 6:
			x = -1; y = 0;
			break;
		case 7:
			x = -1; y = -1;
			break;
		case 8:
			x = 0; y = -1;
			break;
		case 0:
			return v;
		default:
			return NULL;
	}
	if(v->row+x < 0 || v->col+y < 0 || v->row+x >= GRIDWORLD_ROWS || v->col+y >= GRIDWORLD_COLS)
		return NULL;
	if(map[v->row+x][v->col+y].type == BLOCKED)
		return NULL;
	
	return &map[v->row+x][v->col+y];
}

vertex* lpa_succ(vertex* v, int i){
	int x, y;
	switch (i){
		case 1:
			x = 1; y = -1;
			break;
		case 2:
			x = 1; y = 0;	
			break;		
		case 3:
			x = 1; y = 1;
			break;
		case 4:
			x = 0; y = 1;
			break;
		case 5:
			x = -1; y = 1;
			break;
		case 6:
			x = -1; y = 0;
			break;
		case 7:
			x = -1; y = -1;
			break;
		case 8:
			x = 0; y = -1;
			break;
		case 0:
			return v;
		default:
			return NULL;
	}
	if(v->row+x < 0 || v->col+y < 0 || v->row+x >= GRIDWORLD_ROWS || v->col+y >= GRIDWORLD_COLS)
		return NULL;
	if(map[v->row+x][v->col+y].type == START || map[v->row+x][v->col+y].type == BLOCKED)
		return NULL;
	
	if(map[v->row+x][v->col+y].status == UNEXPLORED){
		map[v->row+x][v->col+y].row = v->row+x;
		map[v->row+x][v->col+y].col = v->col+y;
	}
	return &map[v->row+x][v->col+y];
}

vertex* lpa_pred(vertex* v, int i){
	int x = 0, y = 0;
	switch (i){
		case 1:
			x = 1; y = -1;
			break;
		case 2:
			x = 1; y = 0;	
			break;		
		case 3:
			x = 1; y = 1;
			break;
		case 4:
			x = 0; y = 1;
			break;
		case 5:
			x = -1; y = 1;
			break;
		case 6:
			x = -1; y = 0;
			break;
		case 7:
			x = -1; y = -1;
			break;
		case 8:
			x = 0; y = -1;
			break;
		case 0:
			return v;
	}
	if(v->row+x < 0 || v->col+y < 0 || v->row+x >= GRIDWORLD_ROWS || v->col+y >= GRIDWORLD_COLS)
		return NULL;
	
	if(map[v->row+x][v->col+y].status == UNEXPLORED)
		return NULL;
	if(map[v->row+x][v->col+y].type == GOAL || map[v->row+x][v->col+y].type == BLOCKED)
		return NULL;
	return &map[v->row+x][v->col+y];
}
void lpa_calcRHS(vertex* v){
#ifdef EIGHT_CONNECTED_GRIDWORLD
	double result = INF, c, g;
	for(int x = 1; x < 9; ++x){
		vertex* pre = lpa_pred(v, x);
		if(x%2 == 0)
			c = 1.0;
		else
			c = sqrt(2.0);
		if(pre != NULL){
			g = pre->g;
			if(g+c < result){
				result = g+c;
				v->pred = pre;
				v->steps = pre->steps+1;
			}
		}
	}
	v->rhs = result;
#endif
}
void calcKey(vertex *v){
	v->key2 = min(v->rhs, v->g);
	v->key1 = v->key2 + v->h;
}
bool compare_key(vertex* v, vertex* g){
	if(v->key1 < g->key1)
		return true;
	else if(v->key1 > g->key1)
		return false;
	else if(v->key2 >= g->key2)
		return false;
	else 
		return true;
}

void lpa_update(vertex* v, vertex* g){
	if(v->type != START){
		lpa_calcRHS(v);
	}
	if(v->status == ACCESSED)
		lpa_q.remove(v);
	else if(v->status == UNEXPLORED){
		v->g = INF;
		v->h = lpa_heuristic(v, g);
	}
	if(v->g != v->rhs){
		calcKey(v);
		v->status = ACCESSED;
		lpa_q.push_front(v);
	}
}

vector<CellPosition> LPAStar(vertex* start, vertex* goal, bool change){
	 vertex* deq;
	 int num_expanded = 0;
	 bool sol = false;
	 if(!change){
		 if(check_path())
			return lpa_path;
		 start->rhs = 0;
		 start->g = INF;
		 start->h = lpa_heuristic(start, goal);
		 start->steps = 0;
		 calcKey(start);
		 start->status = ACCESSED;
		 goal->rhs = INF;
		 goal->g = INF;
		 cout << "start: (" << start->row << ", " << start->col << ")" << endl;
		 lpa_q.push_front(start);
	 } else {
		 lpa_change();
	 }
	 lpa_path.clear();
	while(!lpa_q.empty()){
		lpa_q.sort(lpa_compare);
		//cout << "list has : " << lpa_q.size() << endl;
		//cout << (lpa_q.front())->row << ", " << (lpa_q.front())->col << endl;
		//cout << lpa_q.back()->row << ", " << lpa_q.back()->col << endl;
		deq = lpa_q.front();
		lpa_q.pop_front();
		deq->status = EXPANDED;
		++num_expanded;
		calcKey(goal);
		if(compare_key(deq, goal) || goal->rhs != goal->g){
			//cout << "expanded: (" << deq->row << ", " << deq->col << ")" << endl;
			if(deq->g > deq->rhs){
				deq->g = deq->rhs;
			} else {
				deq->g = INF;
			}
			for(int i = 0; i < 9; ++i){
				if(i == 0){
					if(deq->g != INF){
						continue;
					}
				}
				vertex* tmp = lpa_succ(deq, i);
				if(deq->pred == tmp || tmp == NULL){
					continue;					
				}
				lpa_update(tmp, goal);
			}
		} else{
			lpa_q.push_front(deq);
			sol = true;
			cout << "Solution found" << endl;
			break;
		}
		
	}
	numberOfExpandedStates = num_expanded;
	vertex* curr = goal;
	cout << "goal: (" << curr->row << ", " << curr->col << ")" << endl;
	CellPosition cp;
	 while(sol){
		 cout << "(" << curr->row << ", " << curr->col << ")" << endl;
		 cp.row = curr->row;
		 cp.col = curr->col;
		 lpa_path.push_back(cp);
		 if(curr->pred != NULL)
			 curr = curr->pred;
		 else 
			 break;
	 }
	 cout << "returning" << endl;
	return lpa_path;
}

void lpa_change(){
	for(int i = 0; i < GRIDWORLD_ROWS; ++i){
		for(int j = 0; j < GRIDWORLD_COLS; ++j){
			if(map[i][j].type != map2[i][j].type){
				map[i][j].type = map2[i][j].type;
				if(map[i][j].status == ACCESSED){
					lpa_q.remove(&map[i][j]);
				}
				for(int k = 1; k < 9; ++k){
					if(lpa_edge(&map[i][j],k) != NULL)
						lpa_update(lpa_edge(&map[i][j],k), goalVertex);
				}
			}
		}
	}
}

bool check_path(){
	if(lpa_path.empty())
		return false;
	CellPosition st = lpa_path.back();
	CellPosition gl = lpa_path.front();
	if(goalVertex->col == gl.col && 
		goalVertex->row == gl.row &&
		startVertex->row == st.row &&
		startVertex->col == st.col){
		return true;
	}
	return false;
}





