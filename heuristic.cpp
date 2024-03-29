#include "heuristic.h"

void Heuristic::init(unsigned int size, unsigned int agents)
{
    h_values.clear();
    h_values.resize(size);
    for(unsigned int xloc = 0; xloc < size; xloc++)
        h_values[xloc].resize(agents, -1);
}

void Heuristic::count(const Map &map, Agent agent)
{
    Node curNode(agent.goal_id, 0, 0, agent.goal_xloc, agent.goal_yloc), newNode;
    open.clear();
    open.insert(curNode);
    while(!open.empty())
    {
        curNode = find_min();
        h_values[curNode.id][agent.id] = curNode.g;
        std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
        for(auto move: valid_moves)
        {
            newNode.xloc = move.xloc;
            newNode.yloc = move.yloc;
            newNode.id = move.id;
            newNode.g = curNode.g + dist(curNode, newNode);
            if(h_values[newNode.id][agent.id] < 0)
            {
                auto it = open.get<1>().find(newNode.id);
                if(it != open.get<1>().end())
                {
                    if(it->g > newNode.g)
                        open.get<1>().erase(it);
                    else
                        continue;
                }
                open.insert(newNode);
            }
        }
    }

}

Node Heuristic::find_min()
{
    Node min = *open.begin();
    open.erase(open.begin());
    return min;
}

