#include "detailed_routing.hpp"

vector<int> v_n = {1, 0, -1};

/*void Small_routing::pin_dcp(){
    for(auto& net:net_list){
        kruskal(&net);
    }
}*/

void Small_routing::initialize(){
    //build grid
    grid.resize(height);
    for(int i=0; i<height; i++){
        grid[i].resize(width);
        for(int j=0; j<width; j++){
           grid[i][j].resize(2);
           for(int k=0; k<2; k++){
                Vertex v(i, j, k);
                grid[i][j][k] = v;
           }
        }
    }
    //mark pin point
    for(auto net:net_list){
        for(int i=0; i<net.pin_list.size(); i++){
            auto pin = net.pin_list[i];
            Vertex* v = &grid[pin.x][pin.y][0];
            v->net_id = net.id;
            v->block = true;
            v->state = -1;
            v->pin_id = i;
        }
    }
    //mark obstacle
    for(auto obst:obstacle_list){
        Vertex* v = &grid[obst.sp.x][obst.sp.y][obst.sp.z];
        while(1){
            v->block = true;
            v->state = -1;
            if(v->coor==obst.ep)
                break;

            if(obst.type==0)
                v = &grid[v->coor.x+1][obst.sp.y][0];
            else if(obst.type==1)
                v = &grid[obst.sp.x][v->coor.y+1][1];
            else
                v = &grid[obst.sp.x][obst.sp.y][obst.ep.z];
        }
    }
}

void Small_routing::T2T_main(){
    int c=0;
    for(auto& net:net_list){
        cout<<"net "<<c<<endl;
        //initial, everypin is a subtree
        for(auto& pin:net.pin_list){
            Subtree* subt = new Subtree(&pin);
            subt->id = net.subt_list.size();
            net.subt_list.push_back(subt);
        }
        //t2t maze routing
        for(int i=0; i<net.pin_list.size()-1; i++){
            int start_id = i;
            for(int j=i; j<net.pin_list.size(); j++){
                if(!net.subt_list[j]->size){
                    start_id = j;
                    break;
                }
                if(net.subt_list[j]->size < net.subt_list[start_id]->size)
                    start_id = j;
            };
            Path* path = new Path;
            T2T_maze_algo(start_id, path, &net);
            int c3=0;
                
            while(!path->overlapped_path.empty()){
                // if(c3==0 && c==71)
                //     break;
                cout<<"net"<<net.id;
                ripup_reroute(path);
                T2T_maze_algo(start_id, path, &net);
                cout<<"c3 "<<c3++<<endl;
                cout<<endl;
            }
            // if(i==1 && c==71){
            //     break;
            // }
            reset_history_cost();
            Coor3D coor_end = path->term.first;
            int end_id = grid[coor_end.x][coor_end.y][coor_end.z].pin_id;
            if(end_id==-1){
                Path* end_p = *(grid[coor_end.x][coor_end.y][coor_end.z].path.begin());
                end_id = end_p->subt_id;
            }
            T2T_maze_post(start_id, end_id, path, &net);
            //cout<<endl;
        }
        // if(c==71){
        //     //Path* p = *(grid[27][18][0].path.begin());
        //     //cout<<p->term.first.x<<" "<<p->term.first.y<<" "<<p->term.first.z<<endl;
        //     //cout<<p->term.second.x<<" "<<p->term.second.y<<" "<<p->term.second.z<<endl;
        //     //cout<<grid[16][7][1].net_id<<endl;
        //     break;
        // }
        c++;
        /*if(grid[33][35][1].block==1 && grid[33][35][1].net_id==-1)
            exit(1);*/
    }
}

void Small_routing::T2T_maze_algo(int start_id , Path* path, Net* net){
    priority_queue <pair<int, Vertex*>, vector<pair<int, Vertex*>>, greater<pair<int, Vertex*>> > pq;
    Subtree* subt_start = net->subt_list[start_id];
    path->subt_id = subt_start->id;
    //initialize, set start and end
    for(auto& pin:subt_start->pin_list){
        Vertex* v = &grid[pin->x][pin->y][0];
        v->dist = 0;
        v->par = v;
        pq.push({0, v});
    }
    for(auto& path:subt_start->path_list){
        for(auto seg:path->seg_list){
            Vertex* v = &grid[seg->sp.x][seg->sp.y][seg->sp.z];
            while(1){
                v->dist = 0;
                v->par = v;
                pq.push({0, v});
                if(v->coor==seg->ep)
                    break;

                if(seg->type==0){
                    int next_x = (seg->sp.x < seg->ep.x)? v->coor.x+1:v->coor.x-1;
                    v = &grid[next_x][seg->sp.y][0];
                }
                else if(seg->type==1){
                    int next_y = (seg->sp.y < seg->ep.y)? v->coor.y+1:v->coor.y-1;
                    v = &grid[seg->sp.x][next_y][1];
                }
                else{
                    v = &grid[seg->sp.x][seg->sp.y][seg->ep.z];
                }      
            }
        }
    }
    for(auto subt_other: net->subt_list){
        if(subt_other->id == subt_start->id)
            continue;
        //cout<<subt_other->id<<endl;
        for(auto& pin:subt_other->pin_list){
            Vertex* v = &grid[pin->x][pin->y][0];
            v->end = true;
        }
        for(auto& path:subt_other->path_list){
            for(auto seg:path->seg_list){
                Vertex* v = &grid[seg->sp.x][seg->sp.y][seg->sp.z];
                while(1){
                    v->end = true;
                    if(v->coor==seg->ep)
                        break;

                    if(seg->type==0){
                        int next_x = (seg->sp.x < seg->ep.x)? v->coor.x+1:v->coor.x-1;
                        v = &grid[next_x][seg->sp.y][0];
                    }
                    else if(seg->type==1){
                        int next_y = (seg->sp.y < seg->ep.y)? v->coor.y+1:v->coor.y-1;
                        v = &grid[seg->sp.x][next_y][1];
                    }
                    else{
                        v = &grid[seg->sp.x][seg->sp.y][seg->ep.z];
                    }      
                }
            }
        }
    }

    //Dijkstra's algo
    while(!pq.empty()){
        auto now = pq.top();
        pq.pop();
        int d = now.first;
        Vertex* v = now.second;
        //find destination
        if(v->end){
            cost_total += d;

            //check overlapped
            Vertex* v_check = v->par;
            while(v_check->par != v_check){
                //cout<<v_check->coor.x<<" "<<v_check->coor.y<<" "<<v_check->coor.z<<endl;
                if(v_check->block){
                    path->overlapped_path.insert(&v_check->coor);
                    //cout<<v_check->coor.x<<" "<<v_check->coor.y<<" "<<v_check->coor.z<<endl;
                    //cout<<"E "<<v_check->path->term.first.x<<" "<<v_check->path->term.first.y<<" "<<v_check->path->term.first.z<<"   ";
                    //cout<<v_check->path->term.second.x<<" "<<v_check->path->term.second.y<<" "<<v_check->path->term.second.z<<endl;
                }
                v_check = v_check->par;
            }
            //cout<<endl;

            if(path->overlapped_path.empty()){
                Segment* seg = new Segment(v->coor);
                seg->type = (v->coor.z == v->par->coor.z)? v->coor.z:2;
                path->term.first = v->coor;
                back_tracking(v, seg, path);
            }
            return;
        }
        for(int k=0; k<3; k++){
            int x_n = (!v->coor.z)? v->coor.x+v_n[k]:v->coor.x;
            int y_n = (!v->coor.z)? v->coor.y:v->coor.y+v_n[k];
            int z_n = (!v->coor.z)? k%2:k%2^1;
            if(x_n==width || x_n==-1 || y_n==height || y_n==-1)
                continue;
            Vertex* next_v = &grid[x_n][y_n][z_n];
            if(next_v->state==-1 && !next_v->end)
                continue;
            int cost = (k%2)? d+via_cost : d+1;
            cost += next_v->history_cost;
            if(next_v->block && !next_v->end && v->path!=next_v->path){
                cost += overlapped_cost;
            }
            if(next_v->dist<0 || cost<next_v->dist){
                next_v->par =v;
                next_v->dist = cost;
                pq.push({cost, next_v});
            }
        }
    }
    cout<<"maze routing failed"<<endl;
    exit(1);
}

void Small_routing::back_tracking(Vertex* v, Segment* seg, Path* path){
    // cout<<v->coor.x<<" "<<v->coor.y<<" "<<v->coor.z<<"   "<<seg->type<<endl;
    seg->ep = v->coor;
    if(seg->type!=2 && v->par->coor.z!=v->coor.z){
        path->seg_list.push_back(seg);
        Segment* seg2 = new Segment(v->coor);
        seg2->type = 2;
        seg = seg2;
    }
    else if(seg->type==2 && (v->par->coor.x!=v->coor.x || v->par->coor.y!=v->coor.y)){
        path->seg_list.push_back(seg);
        Segment* seg2 = new Segment(v->coor);
        seg2->type = (v->par->coor.x!=v->coor.x)? 0:1;
        seg = seg2;
    }

    if(v->par == v){
        path->seg_list.push_back(seg);
        path->term.second = v->coor;
        return;
    }

    if(!v->state){
        v->block = true;
        v->path.insert(path);
        v->state = 1;
    }
    v->par->net_id = v->net_id;
    back_tracking(v->par, seg, path);
}

int Small_routing::ripup_reroute(Path* path){
    rr_total++;
    reset_grid();   //important!!
    vector<int> net_list_rr;
    //rip-up & re-route
    for(auto coor:path->overlapped_path){
        grid[coor->x][coor->y][coor->z].history_cost++;
        //cout<<" "<<coor->x<<" "<<coor->y<<" "<<coor->z<<" "<<grid[coor->x][coor->y][coor->z].path.size()<<endl;
        //while(!grid[coor->x][coor->y][coor->z].path.empty()){
        Path* path_rm = *grid[coor->x][coor->y][coor->z].path.begin();
        if(!path_rm)
            continue;
        int nid = grid[coor->x][coor->y][coor->z].net_id;
        cout<<" overlapped at net"<<nid<<" "<<path_rm->term.first.x<<" "<<path_rm->term.first.y<<" "<<path_rm->term.first.z;
        cout<<"   "<<path_rm->term.second.x<<" "<<path_rm->term.second.y<<" "<<path_rm->term.second.z<<endl;
        net_list_rr.push_back(nid);
        // if(nid==4 && path_rm->term.first.x==7 && path_rm->term.first.y==23 && path_rm->term.second.x==7 && path_rm->term.second.y==17){
        //     //path->overlapped_path.clear();
        //     return -1;
        // }
        remove_path(path_rm);
        // if(nid==4 && path_rm->term.first.x==7 && path_rm->term.first.y==23 && path_rm->term.second.x==7 && path_rm->term.second.y==17){
        //     //path->overlapped_path.clear();
        //     return -1;
        // }
    }
    path->overlapped_path.clear();  
    reverse(net_list_rr.begin(), net_list_rr.end());
    for(int nid:net_list_rr){
        cout<<"reroute "<<nid<<endl;
        Net* net_rr = &net_list[nid];
        Path* path_rr = new Path;
        T2T_maze_algo(0, path_rr, net_rr);
        while(!path_rr->overlapped_path.empty()){
            cout<<"net"<<nid;
            if(ripup_reroute(path_rr)==-1)
                return -1;
            cout<<"reroute "<<nid<<endl;
            T2T_maze_algo(0, path_rr, net_rr);
        }
        Coor3D coor_end = path_rr->term.first;
        int end_id = grid[coor_end.x][coor_end.y][coor_end.z].pin_id;
        if(end_id==-1){
            Path* end_p = *(grid[coor_end.x][coor_end.y][coor_end.z].path.begin());
            end_id = end_p->subt_id;
        }
        T2T_maze_post(0, end_id, path_rr, net_rr);
    }
    
    return 0;
}

void Small_routing::T2T_maze_post(int start_id, int end_id, Path* path, Net* net){
    Subtree* subt_start = net->subt_list[start_id];
    Subtree* subt_end = net->subt_list[end_id];
    //split_path
    Vertex* v1 = &grid[path->term.first.x][path->term.first.y][path->term.first.z];
    //Vertex* v1_oppo = &grid[path->term.first.x][path->term.first.y][path->term.first.z^1];
    Vertex* v2 = &grid[path->term.second.x][path->term.second.y][path->term.second.z];
    //Vertex* v2_oppo = &grid[path->term.second.x][path->term.second.y][path->term.second.z^1];
    if(v1->state==1){
        split_path(subt_end, *v1->path.begin(), v1);
        v1->state = 2;
    }
    v1->path.insert(path);
    
    if(v2->state==1){
        split_path(subt_start, *v2->path.begin(), v2);
        v2->state = 2; 
    }
    v2->path.insert(path);

    reset_grid();
    merge_subt(subt_start, subt_end, path, net->subt_list);
}

void Small_routing::split_path(Subtree* subt, Path* path, Vertex* v_split){
    Path* path2 = new Path(path->subt_id);
    path2->term.first = path->term.first;
    path2->term.second = v_split->coor;
    Coor3D p2f = path->term.first;
    Vertex* v_p2f = &grid[p2f.x][p2f.y][p2f.z];
    set<Path*> path_reg = v_p2f->path;
    //cout<<"KKK"<<endl;
    bool split_seg = false, found = false;
    for(int i=0; i<path->seg_list.size(); i++){
        Segment* seg = path->seg_list[i];
        //cout<<"   "<<seg->sp.x<<" "<<seg->sp.y<<" "<<seg->sp.z<<"   "<<seg->ep.x<<" "<<seg->ep.y<<" "<<seg->ep.z<<"   "<<seg->type<<endl;
        Vertex* v = &grid[seg->sp.x][seg->sp.y][seg->sp.z];
        while(1){
            if(v==v_split){
                if(v->coor==seg->sp);
                else if(v->coor==seg->ep)
                    i++;
                else
                    split_seg = true;
                found = true;
                break;
            }

            v->path = {path2};

            if(v->coor==seg->ep)
                break;
            if(seg->type==0){
                int next_x = (seg->sp.x < seg->ep.x)? v->coor.x+1:v->coor.x-1;
                v = &grid[next_x][seg->sp.y][0];
            }
            else if(seg->type==1){
                int next_y = (seg->sp.y < seg->ep.y)? v->coor.y+1:v->coor.y-1;
                v = &grid[seg->sp.x][next_y][1];
            }
            else{
                v = &grid[seg->sp.x][seg->sp.y][seg->ep.z];
            }      
        }

        if(found){
            path2->seg_list.assign(path->seg_list.begin(), path->seg_list.begin()+i);
            path->seg_list.assign(path->seg_list.begin()+i, path->seg_list.end());
            if(split_seg){
                Segment* seg2 = new Segment(seg);
                seg->sp = v_split->coor;
                seg2->ep = v_split->coor;
                path2->seg_list.push_back(seg2);
            }
            break;
        }
    }
    path->term.first = v_split->coor;

    v_p2f->path = path_reg;
    v_p2f->path.erase(path);
    v_p2f->path.insert(path2);
    v_split->path.insert(path2);
    subt->path_list.push_back(path2);
}

void Small_routing::reset_grid(){
    for(int i=0; i<height; i++){
        for(int j=0; j<width; j++){
            for(int k=0; k<2; k++){
                Vertex* v = &grid[i][j][k];
                v->dist = -1;
                v->par = NULL;
                if(!v->block){
                    v->net_id = -1;
                }
                if(v->end)
                    v->end = false;
            }
        }
    }
}

void Small_routing::reset_history_cost(){
    for(int i=0; i<height; i++){
        for(int j=0; j<width; j++){
            for(int k=0; k<2; k++){
                Vertex* v = &grid[i][j][k];
                v->history_cost = 0;
            }
        }
    }
}

void Small_routing::merge_subt(Subtree* subt_start, Subtree* subt_end, Path* path, vector<Subtree*>& subt_list){
    for(auto pin:subt_end->pin_list){
        subt_start->pin_list.push_back(pin);
    }
    for(auto path_end:subt_end->path_list){
        path_end->subt_id = subt_start->id;
        subt_start->path_list.push_back(path_end);
    }
    subt_start->path_list.push_back(path);
    for(auto& subt:subt_list){
        if(subt==subt_end){
            subt = subt_start;
        }
    }
    int path_len = 0;
    for(auto seg:path->seg_list){
        int seg_len = 0;
        if(seg->type==0)
            seg_len = abs(seg->ep.x - seg->sp.x);
        else if(seg->type==1)
            seg_len = abs(seg->ep.y - seg->sp.y);
        path_len += seg_len;
    }
    subt_start->size += subt_end->size+path_len;
    delete(subt_end);
}

void Small_routing::rec_split_subt(Net* net, Subtree* subt_split, Subtree* subt1, Coor3D trace_point){
    //cout<<trace_point.x<<" "<<trace_point.y<<" "<<trace_point.z<<endl;
    Vertex* v = &grid[trace_point.x][trace_point.y][trace_point.z];
    
    if(v->pin_id!=-1){
        net->subt_list[v->pin_id] = subt1;
        subt1->id = v->pin_id;
        subt1->pin_list.push_back(&v->coor);
    }
    for(auto path:subt_split->path_list){
        //cout<<"  "<<path->term.first.x<<" "<<path->term.first.y<<" "<<path->term.first.z<<endl;
        bool pick = false;
        Coor3D trace_point_next;
        if(path->subt_id!=-1){
            if(path->term.first == trace_point){
                pick = true;
                trace_point_next = path->term.second;
            }
            else if(path->term.second == trace_point){
                pick = true;
                trace_point_next = path->term.first;
            }
        }
        if(pick){
            subt1->path_list.push_back(path);
            path->subt_id = -1;
            rec_split_subt(net, subt_split, subt1, trace_point_next);
        }
    }
}

void Small_routing::remove_path(Path* path_rm){
    //cout<<path_rm->subt_id<<endl;
    Coor3D t1 = path_rm->term.first;
    Coor3D t2 = path_rm->term.second;
    Vertex* v1 = &grid[t1.x][t1.y][t1.z];
    Vertex* v2 = &grid[t2.x][t2.y][t2.z];
    v1->path.erase(path_rm);
    v2->path.erase(path_rm);
    Net* net = &net_list[v1->net_id];
    Subtree* subt_split = net->subt_list[path_rm->subt_id];
    Subtree* subt1 = new Subtree;
    Subtree* subt2 = new Subtree;
    path_rm->subt_id = -1;
    
    //split subtree
    rec_split_subt(net, subt_split, subt1, t1);
    for(auto path:subt_split->path_list){
        if(path->subt_id!=-1){
            subt2->path_list.push_back(path);
        }
    }
    for(int i=0; i<net->subt_list.size(); i++){
        Subtree* subt = net->subt_list[i];
        if(subt==subt_split){
            net->subt_list[i] = subt2;
            subt2->id = i;
            subt2->pin_list.push_back(&net->pin_list[i]);
        }
    }
    for(auto path:subt1->path_list){
        path->subt_id = subt1->id;
    }
    for(auto path:subt2->path_list){
        path->subt_id = subt2->id;
    }
    delete(subt_split);

    //reset grid
    for(auto seg:path_rm->seg_list){
        Vertex* v = &grid[seg->sp.x][seg->sp.y][seg->sp.z];
        while(1){
            if(v!=v1 && v!=v2){
                v->block = 0;
                v->path.clear();
                v->state = 0;
                v->net_id = -1;
            }

            if(v->coor==seg->ep)
                break;

            if(seg->type==0){
                int next_x = (seg->sp.x < seg->ep.x)? v->coor.x+1:v->coor.x-1;
                v = &grid[next_x][seg->sp.y][0];
            }
            else if(seg->type==1){
                int next_y = (seg->sp.y < seg->ep.y)? v->coor.y+1:v->coor.y-1;
                v = &grid[seg->sp.x][next_y][1];
            }
            else{
                v = &grid[seg->sp.x][seg->sp.y][seg->ep.z];
            }      
        }
    }

    //merge paths
    cout<<v1->state<<" "<<v2->state<<endl;
    if(v1->state == 2){
        merge_path(subt1, v1);
        v1->state = 1;
    }
    if(v2->state == 2){
        merge_path(subt2, v2);
        v2->state = 1;
    }

    delete(path_rm);
}

void Small_routing::merge_path(Subtree* subt, Vertex* v_merge){
    cout<<"merge at "<<v_merge->coor.x<<" "<<v_merge->coor.y<<" "<<v_merge->coor.z<<endl;
    Path* path1 = NULL;
    Path* path2 = NULL;
    bool path1_sp, path2_sp;    //0:start point  1:end point
    for(int i=0; i<subt->path_list.size(); i++){
        auto path = subt->path_list[i];
        Coor3D t1 = path->term.first;
        Coor3D t2 = path->term.second;
        if(path1 && path2){
            subt->path_list[i-1] = subt->path_list[i];
        }
        else if(t1 == v_merge->coor || t2 == v_merge->coor){
            if(!path1){
                path1 = path;
                path1_sp = (t1 == v_merge->coor)? 0:1;
            }
            else{
                path2 = path;
                path2_sp = (t1 == v_merge->coor)? 0:1;
            }
        }
    }
    subt->path_list.pop_back();
    Segment* seg1 = (!path1_sp)? path1->seg_list.front() : path1->seg_list.back();
    Segment* seg2 = (!path2_sp)? path2->seg_list.front() : path2->seg_list.back();
    if(!path1_sp)
        path1->term.first = (path2_sp)? path2->term.first : path2->term.second;
    else
        path1->term.second = (path2_sp)? path2->term.first : path2->term.second;
    if(!path1_sp){
        reverse(path1->seg_list.begin(), path1->seg_list.end());
        swap(path1->term.first, path1->term.second);
        for(auto seg: path1->seg_list)
            swap(seg->sp, seg->ep);
    }
    if(path2_sp){
        reverse(path2->seg_list.begin(), path2->seg_list.end());
        for(auto seg: path2->seg_list)
            swap(seg->sp, seg->ep);
    }
    
    Coor3D p2a = (path2_sp)? path2->term.first:path2->term.second;
    Vertex* v_p2a = &grid[p2a.x][p2a.y][p2a.z];
    set<Path*> path_reg = v_p2a->path;
    for(auto seg:path2->seg_list){
        Vertex* v = &grid[seg->sp.x][seg->sp.y][seg->sp.z];
        while(1){
            v->path = {path1};

            if(v->coor==seg->ep)
                break;

            if(seg->type==0){
                int next_x = (seg->sp.x < seg->ep.x)? v->coor.x+1:v->coor.x-1;
                v = &grid[next_x][seg->sp.y][0];
            }
            else if(seg->type==1){
                int next_y = (seg->sp.y < seg->ep.y)? v->coor.y+1:v->coor.y-1;
                v = &grid[seg->sp.x][next_y][1];
            }
            else{
                v = &grid[seg->sp.x][seg->sp.y][seg->ep.z];
            }      
        }
        //merge seg
        if(seg==seg2 && seg1->type==seg2->type){
            seg1->ep = seg->ep;
            delete(seg2);
        }
        else
            path1->seg_list.push_back(seg);
    }
    v_p2a->path = path_reg;
    v_p2a->path.erase(path2);
    v_p2a->path.insert(path1);

    delete(path2);

}

void Small_routing::post_proc(){
    for(auto& net:net_list){
        if(!net.subt_list.size())
            break;
        /*if(net.id!=63)
            continue;*/
        //cout<<"net "<<net.id<<endl;
        int c=0;
        //for(auto subt:net.subt_list){
        //cout<<"subt "<<subt->id<<" "<<subt->size<<endl;
        if(net.id==62)
            c=1;
        for(auto path:net.subt_list[c]->path_list){
        //for(auto path:subt->path_list){
            //cout<<"E "<<path->term.first.x<<" "<<path->term.first.y<<" "<<path->term.first.z<<"  "<<grid[path->term.first.x][path->term.first.y][path->term.first.z].path.size()<<"   ";
            //cout<<path->term.second.x<<" "<<path->term.second.y<<" "<<path->term.second.z<<"  "<<grid[path->term.second.x][path->term.second.y][path->term.second.z].path.size()<<"        "<<path->subt_id<<endl;
            for(int i=0; i<path->seg_list.size(); i++){
                Segment* seg = path->seg_list[i];
                //cout<<"   "<<seg->sp.x<<" "<<seg->sp.y<<" "<<seg->sp.z<<"   "<<seg->ep.x<<" "<<seg->ep.y<<" "<<seg->ep.z<<"   "<<seg->type<<endl;
                int seg_len = 0;
                if(seg->type==0){
                    if(seg->sp.x > seg->ep.x)
                        swap(seg->sp.x, seg->ep.x);
                    net.hseg_list.push_back(seg);
                    seg_len = seg->ep.x - seg->sp.x;
                }   
                else if(seg->type==1){
                    if(seg->sp.y > seg->ep.y)
                        swap(seg->sp.y, seg->ep.y);
                    net.vseg_list.push_back(seg);
                    seg_len = seg->ep.y - seg->sp.y;
                }
                else{
                    net.via_list.insert({seg->sp.x, seg->sp.y});
                }
                WL_total += seg_len;
            }
        }
        //}
        via_total += net.via_list.size();
        //cout<<endl;
    }
    cost_total = WL_total+via_total;
}




/*int Disjoint_set::find(int x){
    if(x != par[x])
        par[x] = find(par[x]);
    return par[x];
}

void Disjoint_set::merge(int x, int y){
    x = find(x), y = find(y);
    if(rank[x]>=rank[y])
        par[y] = x;
    else
        par[x] = y;
    if(rank[x]==rank[y])
        rank[x]++;
}  

void kruskal(Net* net){
    vector<pair<int, pii>> paths;
    int num = net->pin_list.size();
    for(int i=0; i<num; i++){
        for(int j=i+1; j<num; j++){
            int manh_dist = abs(net->pin_list[i].x-net->pin_list[j].x)+abs(net->pin_list[i].y-net->pin_list[j].y);
            paths.push_back({manh_dist, {i, j}});
        }
    }
    sort(paths.begin(),  paths.end());
    Disjoint_set ds(num);
    for(auto path:paths){
        int x = path.second.first, y = path.second.second;
        int ds_x = ds.find(x);
        int ds_y = ds.find(y);
        if(ds_x!=ds_y){
            net->pin_pair.push_back({x, y});
            ds.merge(ds_x, ds_y);
        }
    }
}*/