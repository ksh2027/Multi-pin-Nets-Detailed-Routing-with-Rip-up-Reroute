#pragma once
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include "detailed_routing.hpp"

std::stringstream ss;

namespace io{

void tokenLine(std::vector<std::string> &tokens, std::string line){
    ss.clear(); ss.str(line);
    tokens.clear(); tokens.resize(0);
    std::string intermediate;
    while(true){
        ss >> intermediate;
        if(ss.fail()) break;
        tokens.push_back(intermediate);
    }
}

void readLayout(Small_routing *layout, char const *file_path)
{
    std::ifstream in_file(file_path);
    std::string line;
    std::vector<std::string> tokens;
    while(getline(in_file, line)){
        tokenLine(tokens, line);
        if(tokens.size() == static_cast<unsigned int>(2)){
            if(tokens[0] == "Width"){
                layout->width = stoi(tokens[1]);
            }
            else if(tokens[0] == "Height"){
                layout->height = stoi(tokens[1]);
            }
            else if(tokens[0] == "Layer"){
                layout->num_of_layers = stoi(tokens[1]);
            }
            else if(tokens[0] == "Obstacle_num"){
                layout->obstacle_list.resize(stoi(tokens[1]));
                for(unsigned i = 0; i < layout->obstacle_list.size(); i++){
                    getline(in_file, line); tokenLine(tokens, line);
                    Segment seg(Coor3D {stoi(tokens[0]), stoi(tokens[1]), stoi(tokens[2])});
                    seg.ep = Coor3D {stoi(tokens[3])-1, stoi(tokens[4])-1, stoi(tokens[5])};
                    seg.type = stoi(tokens[2]);
                    layout->obstacle_list.at(i) = seg;
                }
            }
            else if(tokens[0] == "Net_num"){
                layout->net_list.resize(stoi(tokens[1]));
                for(unsigned i = 0; i < layout->net_list.size(); i++){
                    Net tmp_net;
                    while(getline(in_file, line)){
                        tokenLine(tokens, line);
                        if(tokens.size() == static_cast<unsigned int>(2) && tokens[0] == "Net_id") break;
                    }
                    tmp_net.id = stoi(tokens[1]);
                    while(getline(in_file, line)){
                        tokenLine(tokens, line);
                        if(tokens.size() == static_cast<unsigned int>(2) && tokens[0] == "pin_num") break;
                    }
                    tmp_net.pin_list.resize(stoi(tokens[1]));
                    for(unsigned j = 0; j < tmp_net.pin_list.size(); j++){
                        getline(in_file, line); tokenLine(tokens, line);
                        tmp_net.pin_list.at(j) = Coor3D{stoi(tokens[0]), stoi(tokens[1]), stoi(tokens[2])};
                    }
                    layout->net_list.at(i) = tmp_net;
                }
            }
            else if(tokens[0] == "Via_cost"){
                layout->via_cost = stoi(tokens[1]);
            }
            else if(tokens[0] == "Horizontal_segment_cost"){
                layout->horizontal_segment_cost = stoi(tokens[1]);
            }
            else if(tokens[0] == "Vertical_segment_cost"){
                layout->vertical_segment_cost = stoi(tokens[1]);
            }
        }
    }
}
void writeLayout(Small_routing *layout, char const *file_path)
{
    std::ofstream out_file(file_path, std::ofstream::trunc);
    out_file << "Width " << layout->width << "\n";
    out_file << "Height " << layout->height << "\n";
    out_file << "Layer " << layout->num_of_layers << "\n";
    out_file << "Total_cost " << layout->cost_total << "\n";
    out_file << "Total_WL " << layout->WL_total << "\n";
    out_file << "Total_#via " << layout->via_total << "\n";
    out_file << "Obstacle_num " << layout->obstacle_list.size() << "\n";
    for(unsigned i = 0; i < layout->obstacle_list.size(); i++){
        Segment* seg = &layout->obstacle_list.at(i);
        out_file << seg->sp.x << " " << seg->sp.y << " " << seg->sp.z << " ";
        out_file << seg->ep.x+1 << " " << seg->ep.y+1 << " " << seg->sp.z << " ";
        out_file << "\n";
    }
    out_file << "Net_num " << layout->net_list.size() << "\n";
    for(unsigned i = 0; i < layout->net_list.size(); i++){
        Net* net = &layout->net_list.at(i);
        out_file << "Net_id " << net->id << "\n";
        out_file << "pin_num " << net->pin_list.size() << "\n";
        for(unsigned j = 0; j < net->pin_list.size(); j++){
            out_file << net->pin_list.at(j).x << " " << net->pin_list.at(j).y << " " << net->pin_list.at(j).z << "\n";
        }
        out_file << "Via_num " << net->via_list.size() << "\n";
        for(auto via : net->via_list){
            out_file << via.first << " " << via.second << "\n";
        }
        out_file << "H_segment_num " << net->hseg_list.size() << "\n";
        for(unsigned j = 0; j < net->hseg_list.size(); j++){
            Segment* seg = net->hseg_list.at(j);
            out_file << seg->sp.x << " " << seg->sp.y << " " << seg->sp.z << " ";
            out_file << seg->ep.x+1 << " " << seg->ep.y+1 << " " << seg->sp.z << " ";
            out_file << "\n";
        }
        out_file << "V_segment_num " << net->vseg_list.size() << "\n";
        for(unsigned j = 0; j < net->vseg_list.size(); j++){
            Segment* seg = net->vseg_list.at(j);
            out_file << seg->sp.x << " " << seg->sp.y << " " << seg->sp.z << " ";
            out_file << seg->ep.x+1 << " " << seg->ep.y+1 << " " << seg->sp.z << " ";
            out_file << "\n";
        }
        // out_file << "color_1 " << 0 << "\n";
        // out_file << "color_2 " << 0 << "\n";
    }
}

void writeOutput(Small_routing *layout, char const *file_path)
{
    std::ofstream out_file(file_path, std::ofstream::app);
    out_file << "Total_cost " << layout->cost_total << "\n";
    out_file << "Total_WL " << layout->WL_total << "\n";
    out_file << "Total_#via " << layout->via_total << "\n";
    out_file << "Total_#reroute " << layout->rr_total << "\n";
    out_file << "Time " << layout->time_total << "\n";
    out_file << endl;
}
}