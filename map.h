#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <vector>
#include "tinyxml2.h"
#include "const.h"
#include "structs.h"

class Map
{
private:
    std::vector<gNode> nodes;
    std::vector<std::vector<Node>> valid_moves;
    int  size;
    int  connectedness;
    double agent_size;
    bool check_line(int x1, int y1, int x2, int y2);
    
public:
    Map(double size, int k){ agent_size = size; connectedness = k; }
    ~Map(){}
    int  get_size() const { return size; }
    bool get_roadmap(const char* FileName);
    bool hasNode(int xloc, int yloc) const;
    gNode get_gNode(int id) const {if(id < int(nodes.size())) return nodes[id]; return gNode();}
    int  get_id(int xloc, int yloc) const;
    double get_i (int id) const;
    double get_j (int id) const;
    std::vector<Node> get_valid_moves(int id) const;
};

#endif // MAP_H
