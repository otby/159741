#include <iostream>
#include <list>

using namespace std;
struct vector{
	int i;
};

vector* astar_succ(vector* v, short i){
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
	v->i = x;
	return v;
}
list<vector*> lpa_q;
void lpa_remove(vector* v){
	list<vector*>::iterator it;
	it=lpa_q.begin();
	for (unsigned int i = 0; i<lpa_q.size(); ++i){
		cout << (*it)->i << endl;
		++it;
	}
	
}

int main(void){
	vector a,b,c,d,e;
	vector* v = &e;
	vector* w = &a;
	vector* x = &b;
	vector* y = &c;
	vector* z = &d;
	v->i = 1;
	w->i = 2;
	x->i = 3;
	y->i = 4;
	z->i = 5;
	lpa_q.push_back(v);
	lpa_q.push_back(w);
	lpa_q.push_back(x);
	lpa_q.push_back(y);
	lpa_q.push_back(z);
	v = astar_succ(v, 4);
	cout << v->i << " " << y->i << " " << z->i << endl;
	cout << lpa_q.size() << endl;
	lpa_remove(v);
	cout << lpa_q.size() << endl;
	
	return 0;

}
