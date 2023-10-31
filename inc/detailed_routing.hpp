#pragma once
#include "data_struct.hpp"
#include <vector>
#include <iostream>
#include <algorithm>
#include <queue>
#include <unordered_map>

class Small_routing{
public:
    int width, height;
	int num_of_layers = 2;
	int via_cost = 1;
	int horizontal_segment_cost = 1, vertical_segment_cost = 1;
    int overlapped_cost = 10;
	int cost_total=0, WL_total=0, via_total=0, rr_total=0;
    double time_total=0;
	vector<Segment> obstacle_list;
	vector<Net> net_list;
    vector<vector<vector<Vertex>>> grid;
    Small_routing(){}
    ~Small_routing(){}
    void initialize();
    //void pin_dcp();
    void T2T_main();
    void T2T_maze_algo(int start_id, Path* path, Net* net);
    void back_tracking(Vertex* v, Segment* seg, Path* path);
    int ripup_reroute(Path* path);
    void T2T_maze_post(int start_id, int end_id, Path* path, Net* net);
    void split_path(Subtree* subt, Path* path, Vertex* v);
    void reset_grid();
    void reset_history_cost();
    void merge_subt(Subtree* subt_start, Subtree* subt_end, Path* path, vector<Subtree*>& subtlist);
    void rec_split_subt(Net* net, Subtree* subt_split, Subtree* subt1, Coor3D trace_point);
    void remove_path(Path* path_rm);
    void merge_path(Subtree* subt, Vertex* v_merge);
    void post_proc();

};

/*class Disjoint_set{
public:
    vector<int> par, rank;

    Disjoint_set(){}
    Disjoint_set(int num){
        rank.assign(num+1, 0);
        par.resize(num+1);
        for(int i=0; i<=num; i++)
            par[i] = i;
    }
    ~Disjoint_set(){}
    int find(int x);
    void merge(int x, int y);
};*/

//void kruskal(Net* net);
