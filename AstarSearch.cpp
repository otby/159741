#include "AstarSearch.h"
 
vector<CellPosition> astar_path;
priority_queue<vertex*,vector<vertex*>,astar_compare> astar_q;


double astar_heuristic(vertex *s, vertex *g){
	return sqrt((s->row-g->row)*(s->row-g->row)+(s->col-g->col)*(s->col-g->col));
}

vertex* astar_succ(vertex* v, short i){
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
			cerr << "Unrecognised successor" << endl;
			return NULL;
	}
	if(v->row+x < 0 || v->col+y < 0 || v->row+x >= GRIDWORLD_ROWS || v->col+y >= GRIDWORLD_COLS){
		cout << "out of bounds" << endl;
		return NULL;
	}
	
	if(map[v->row+x][v->col+y].status == UNEXPLORED){
		map[v->row+x][v->col+y].row = v->row+x;
		map[v->row+x][v->col+y].col = v->col+y;
	}
	return &map[v->row+x][v->col+y];
}

double cost(vertex *s, vertex *g){
	if(s->row == g->row || s->col == g->col)
		return 1.0;
	else 
		return sqrt(2); // SQRT2;
}

 vector<CellPosition> a_star(vertex* start, vertex* goal){
	 astar_path.clear();
	 vertex* deq;
	 int num_expanded = 0, num_accesses = 0;
	 bool sol = false;
	 start->g = 0;
	 start->h = astar_heuristic(start, goal);
	 start->f = start->g+start->h;
	 start->status = ACCESSED;
	 cout << "start: (" << start->row << ", " << start->col << ")" << endl;
	 astar_q.push(start);
	 while(!astar_q.empty()){
		 deq = astar_q.top();
		 astar_q.pop();
		 ++num_expanded;
		 if(deq->status == EXPANDED){
			 continue;
		 } else {
			deq->status = EXPANDED; 
		 }
		 //cout << "deq: (" << deq->row << ", " << deq->col << ")" << endl;
		 if(deq->type == GOAL){
			 cout << "Found solution" << endl;
			 numberOfExpandedStates = num_expanded;
			 numberOfVertexAccesses = num_accesses;
			 sol = true;
			 break;
		 }
#ifdef EIGHT_CONNECTED_GRIDWORLD
		 for(short i = 1; i < 9; ++i){ // start at 1 to skip itself.
			 vertex* tmp = astar_succ(deq, i);
			 if(tmp == NULL)
				continue;
			 if(deq->pred == tmp){
				continue;
			 }
			 if((tmp->type == TRAVERSABLE || tmp->type == GOAL) && tmp->status == UNEXPLORED){
				 tmp->status = ACCESSED;
				 tmp->g = deq->g+cost(deq, tmp);
				 tmp->h = astar_heuristic(tmp, goal);
				 tmp->f = tmp->g+tmp->h;
				 tmp->pred = deq;
				 tmp->steps = deq->steps+1;
				 astar_q.push(tmp);
				 ++num_accesses;
				 //cout << "\tpush: (" << tmp->row << ", " << tmp->col << ")" << endl;
			 } else if(tmp->status == ACCESSED) {
				 double tmp_g = deq->g+cost(deq, tmp);
				 ++num_accesses;
				 if(tmp_g < tmp->g){
					 tmp->g = tmp_g;
					 tmp->h = astar_heuristic(tmp, goal);
					 tmp->f = tmp->g+tmp->h;
					 tmp->pred = deq;
					 astar_q.push(tmp);
					 //cout << "\toverride: (" << tmp->row << ", " << tmp->col << ")" << endl;
				 }
			 } // else ignore
		 }
#endif
	 }
	 numberOfExpandedStates = num_expanded;
	 vertex* curr = goal;
	 cout << "goal: (" << curr->row << ", " << curr->col << ")" << endl;
	 CellPosition cp;
	 while(sol){
		 cp.row = curr->row;
		 cp.col = curr->col;
		 astar_path.push_back(cp);
		 if(curr->pred != NULL)
			 curr = curr->pred;
		 else 
			 break;
	 }
	 while(!astar_q.empty()){
		 astar_q.pop();
	 }
	 return astar_path;
 }


 
 
 /*
	 vertex a, b, c;
	 a.f = 1.0;
	 a.type = 'a';
	 b.f = 1.0;
	 b.type = 'b';
	 c.f = 2.0;
	 c.type = 'c';
	 
	 astar_q.push(c);
	 astar_q.push(a);
	 astar_q.push(b);
	 
				 cout << "\n\n*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+\n" << endl;
				 cout << "(" << deq->row+x << ", " << deq->col+y << ") type: "<< tmp->type << " status: " << tmp->status << endl;
	 cout << "\n\n*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+\n" << endl;
	 cout << (*map)[11][5] << endl;
 
	 cout << "\n\n*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+\n" << endl;
	 
	 cout << astar_q.top().type << " ";
	 astar_q.pop();
	 cout	<< astar_q.top().type << " ";
	 astar_q.pop();
    cout << astar_q.top().type << endl;
	 astar_q.pop();
	 cout << "\n*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+\n\n" << endl;;
	 */
