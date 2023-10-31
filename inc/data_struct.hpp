#pragma once
#include <vector>
#include <set>
using namespace std;
typedef pair<int, int> pii;

class Coor3D{
public:
	int x, y, z;
	Coor3D(){}
	Coor3D(int _x, int  _y, int  _z) : x(_x), y(_y), z(_z) {}
	bool operator==(Coor3D & coor) const{
		return this->x == coor.x && this->y == coor.y && this->z == coor.z;
	}
	~Coor3D(){}
};
class Segment{
public:
    int type; //0:horizental 1:vertical 2:z-direction
	Coor3D sp;
	Coor3D ep;
	Segment(){}
	Segment(Coor3D _sp) : sp(_sp){}
	Segment(Segment* seg) : sp(seg->sp), ep(seg->ep), type(seg->type) {}
	~Segment(){}
};
class Path{
public:
    vector<Segment*> seg_list;
    pair<Coor3D, Coor3D> term;
	set<Coor3D*> overlapped_path;
	int subt_id;  //for rip-up & re-route
	Path(){}
    Path(int id) : subt_id(id) {
		seg_list.resize(0);
		overlapped_path.clear();
	}
    ~Path(){}
};
class Subtree{
public:
	int id;
	int size = 0;
    vector<Path*> path_list;
    vector<Coor3D*> pin_list;
    Subtree(){}
    Subtree(Coor3D* pin){
        pin_list.push_back(pin);
        path_list.resize(0);
    }
    ~Subtree(){}
};
class Net{
public:
	int id;
	vector<Coor3D> pin_list;
	vector<pii> pin_pair;
	set<pii> via_list;
	vector<Subtree*> subt_list;
	vector<Segment*> hseg_list;
	vector<Segment*> vseg_list;
	Net(){
		id = -1;
		this->pin_list.resize(0);
		this->pin_pair.resize(0);
		this->subt_list.resize(0);
		this->hseg_list.resize(0);
		this->vseg_list.resize(0);
	}
	~Net(){}
};
class Vertex{
public:
    Coor3D coor;
	int net_id = -1;
	int pin_id = -1;  //for pins
    bool block = false;
    int state = 0; //-1:pin/obstacle 0:space 1:line 2:steiner node
    set<Path*> path; //only for state=1
	//for Dijkstra's algo
    int dist = -1;
    Vertex* par;
	bool end = false;
	int history_cost = 0;
    Vertex(){}
    Vertex(int _x, int _y, int _z) : coor(_x, _y, _z) {}
    ~Vertex(){}
};