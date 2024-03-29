#include "sipp.h"

void SIPP::clear()
{
    open.clear();
    close.clear();
    collision_intervals.clear();
    landmarks.clear();
    constraints.clear();
    visited.clear();
    path.cost = -1;
}

double SIPP::dist(const Node& a, const Node& b)
{
    return std::sqrt(pow(a.xloc - b.xloc, 2) + pow(a.yloc - b.yloc, 2));
}

void SIPP::find_successors(Node curNode, const Map &map, std::list<Node> &succs, Heuristic &h_values, Node goal)
{
    Node newNode;
    std::vector<Node> valid_moves = map.get_valid_moves(curNode.id);
    for(auto move : valid_moves)
    {
        newNode.xloc = move.xloc;
        newNode.yloc = move.yloc;
        newNode.id = move.id;
        double cost = dist(curNode, newNode);
        newNode.g = curNode.g + cost;
        std::vector<std::pair<double, double>> intervals(0);
        auto colls_it = collision_intervals.find(newNode.id);
        if(colls_it != collision_intervals.end())
        {
            std::pair<double, double> interval = {0, CN_INFINITY};
            for(unsigned int xloc = 0; xloc < colls_it->second.size(); xloc++)
            {
                interval.second = colls_it->second[xloc].first;
                intervals.push_back(interval);
                interval.first = colls_it->second[xloc].second;
            }
            interval.second = CN_INFINITY;
            intervals.push_back(interval);
        }
        else
            intervals.push_back({0, CN_INFINITY});
        auto cons_it = constraints.find({curNode.id, newNode.id});
        int id(0);
        for(auto interval: intervals)
        {
            newNode.interval_id = id;
            id++;
            auto it = visited.find(newNode.id + newNode.interval_id * map.get_size());
            if(it != visited.end())
                if(it->second.second)
                    continue;
            if(interval.second < newNode.g)
                continue;
            if(interval.first > newNode.g)
                newNode.g = interval.first;
            if(cons_it != constraints.end())
                for(unsigned int xloc = 0; xloc < cons_it->second.size(); xloc++)
                    if(newNode.g - cost + CN_EPSILON > cons_it->second[xloc].t1 && newNode.g - cost < cons_it->second[xloc].t2)
                        newNode.g = cons_it->second[xloc].t2 + cost;
            newNode.interval = interval;
            if(newNode.g - cost > curNode.interval.second || newNode.g > newNode.interval.second)
                continue;
            if(it != visited.end())
            {
                if(it->second.first - CN_EPSILON < newNode.g)
                    continue;
                else
                    it->second.first = newNode.g;
            }
            else
                visited.insert({newNode.id + newNode.interval_id * map.get_size(), {newNode.g, false}});
            if(goal.id == agent.goal_id) //perfect heuristic is known
                newNode.f = newNode.g + h_values.get_value(newNode.id, agent.id);
            else
            {
                double h = sqrt(pow(goal.xloc - newNode.xloc, 2) + pow(goal.yloc - newNode.yloc, 2));
                for(unsigned int xloc = 0; xloc < h_values.get_size(); xloc++) //differential heuristic with pivots placed to agents goals
                    h = std::max(h, fabs(h_values.get_value(newNode.id, xloc) - h_values.get_value(goal.id, xloc)));
                newNode.f = newNode.g + h;
            }
            succs.push_back(newNode);
        }
    }
}

Node SIPP::find_min()
{
    Node min = *open.begin();
    open.pop_front();
    return min;
}

void SIPP::add_open(Node newNode)
{
    if (open.empty() || open.back().f - CN_EPSILON < newNode.f)
    {
        open.push_back(newNode);
        return;
    }
    for(auto iter = open.begin(); iter != open.end(); ++iter)
    {
        if(iter->f > newNode.f + CN_EPSILON) // if newNode.f has lower f-value
        {
            open.insert(iter, newNode);
            return;
        }
        else if(fabs(iter->f - newNode.f) < CN_EPSILON && newNode.g + CN_EPSILON > iter->g) // if f-values are equal, compare g-values
        {
            open.insert(iter, newNode);
            return;
        }
    }
    open.push_back(newNode);
    return;
}

std::vector<Node> SIPP::reconstruct_path(Node curNode)
{
    path.nodes.clear();
    if(curNode.parent != nullptr)
    do
    {
        path.nodes.insert(path.nodes.begin(), curNode);
        curNode = *curNode.parent;
    }
    while(curNode.parent != nullptr);
    path.nodes.insert(path.nodes.begin(), curNode);
    for(unsigned int xloc = 0; xloc < path.nodes.size(); xloc++)
    {
        unsigned int yloc = xloc + 1;
        if(yloc == path.nodes.size())
            break;
        if(fabs(path.nodes[yloc].g - path.nodes[xloc].g - dist(path.nodes[yloc], path.nodes[xloc])) > CN_EPSILON)
        {
            Node add = path.nodes[xloc];
            add.g = path.nodes[yloc].g - dist(path.nodes[yloc], path.nodes[xloc]);
            path.nodes.emplace(path.nodes.begin() + yloc, add);
        }
    }
    return path.nodes;
}

void SIPP::add_collision_interval(int id, std::pair<double, double> interval)
{
    std::vector<std::pair<double, double>> intervals(0);
    if(collision_intervals.count(id) == 0)
        collision_intervals.insert({id, {interval}});
    else
        collision_intervals[id].push_back(interval);
    std::sort(collision_intervals[id].begin(), collision_intervals[id].end());
    for(unsigned int xloc = 0; xloc + 1 < collision_intervals[id].size(); xloc++)
        if(collision_intervals[id][xloc].second + CN_EPSILON > collision_intervals[id][xloc+1].first)
        {
            collision_intervals[id][xloc].second = collision_intervals[id][xloc+1].second;
            collision_intervals[id].erase(collision_intervals[id].begin() + xloc + 1);
            xloc--;
        }
}

void SIPP::add_move_constraint(Move move)
{
    std::vector<Move> m_cons(0);
    if(constraints.count({move.id1, move.id2}) == 0)
        constraints.insert({{move.id1, move.id2}, {move}});
    else
    {
        m_cons = constraints.at({move.id1, move.id2});
        bool inserted(false);
        for(unsigned int xloc = 0; xloc < m_cons.size(); xloc++)
        {
            if(inserted)
                break;
            if(m_cons[xloc].t1 > move.t1)
            {
                if(m_cons[xloc].t1 < move.t2 + CN_EPSILON)
                {
                    m_cons[xloc].t1 = move.t1;
                    if(move.t2 + CN_EPSILON > m_cons[xloc].t2)
                        m_cons[xloc].t2 = move.t2;
                    inserted = true;
                    if(xloc != 0)
                        if(m_cons[xloc-1].t2 + CN_EPSILON > move.t1 && m_cons[xloc-1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[xloc-1].t2 = move.t2;
                            if(m_cons[xloc-1].t2 + CN_EPSILON > m_cons[xloc].t1 && m_cons[xloc-1].t2 < m_cons[xloc].t2 + CN_EPSILON)
                            {
                                m_cons[xloc-1].t2 = m_cons[xloc].t2;
                                m_cons.erase(m_cons.begin() + xloc);
                            }
                            inserted = true;
                        }
                }
                else
                {
                    if(xloc != 0)
                        if(m_cons[xloc-1].t2 + CN_EPSILON > move.t1 && m_cons[xloc-1].t2 < move.t2 + CN_EPSILON)
                        {
                            m_cons[xloc-1].t2 = move.t2;
                            inserted = true;
                            break;
                        }
                    m_cons.insert(m_cons.begin() + xloc, move);
                    inserted = true;
                }
            }
        }
        if(m_cons.back().t2 + CN_EPSILON > move.t1 && m_cons.back().t2 < move.t2 + CN_EPSILON)
            m_cons.back().t2 = move.t2;
        else if(!inserted)
            m_cons.push_back(move);
        constraints.at({move.id1, move.id2}) = m_cons;
    }
}

void SIPP::make_constraints(std::list<Constraint> &cons)
{
    for(auto con : cons)
    {
        if(con.positive == false)
        {
            if(con.id1 == con.id2) // wait consatraint
                add_collision_interval(con.id1, std::make_pair(con.t1, con.t2));
            else
                add_move_constraint(Move(con));
        }
        else
        {
            bool inserted = false;
            for(unsigned int xloc = 0; xloc < landmarks.size(); xloc++)
                if(landmarks[xloc].t1 > con.t1)
                {
                    landmarks.insert(landmarks.begin() + xloc, Move(con.t1, con.t2, con.id1, con.id2));
                    inserted = true;
                    break;
                }
            if(!inserted)
                landmarks.push_back(Move(con.t1, con.t2, con.id1, con.id2));
        }
    }
}

Path SIPP::add_part(Path result, Path part)
{
    part.nodes.erase(part.nodes.begin());
    for(auto n: part.nodes)
        result.nodes.push_back(n);
    return result;
}

std::vector<Path> SIPP::find_partial_path(std::vector<Node> starts, std::vector<Node> goals, const Map &map, Heuristic &h_values, double max_f)
{
    open.clear();
    close.clear();
    path.cost = -1;
    visited.clear();
    std::vector<Path> paths(goals.size());
    int pathFound(0);
    for(auto s:starts)
    {
        s.parent = nullptr;
        open.push_back(s);
        visited.insert({s.id + s.interval_id * map.get_size(), {s.g, false}});
    }
    Node curNode;
    while(!open.empty())
    {
        curNode = find_min();
        auto v = visited.find(curNode.id + curNode.interval_id * map.get_size());
        if(v->second.second)
            continue;
        v->second.second = true;
        auto parent = &close.insert({curNode.id + curNode.interval_id * map.get_size(), curNode}).first->second;
        if(curNode.id == goals[0].id)
        {
            for(unsigned int xloc = 0; xloc < goals.size(); xloc++)
                if(curNode.g - CN_EPSILON < goals[xloc].interval.second && goals[xloc].interval.first - CN_EPSILON < curNode.interval.second)
                {
                    paths[xloc].nodes = reconstruct_path(curNode);
                    if(paths[xloc].nodes.back().g < goals[xloc].interval.first)
                    {
                        curNode.g = goals[xloc].interval.first;
                        paths[xloc].nodes.push_back(curNode);
                    }
                    paths[xloc].cost = curNode.g;
                    paths[xloc].expanded = int(close.size());
                    pathFound++;
                }
            if(pathFound == int(goals.size()))
                return paths;
        }
        std::list<Node> succs;
        succs.clear();
        find_successors(curNode, map, succs, h_values, Node(goals[0].id, 0, 0, goals[0].xloc, goals[0].yloc));
        std::list<Node>::iterator it = succs.begin();
        while(it != succs.end())
        {
            if(it->f > max_f)
            {
                it++;
                continue;
            }
            it->parent = parent;
            add_open(*it);
            it++;
        }
    }
    return paths;
}

std::vector<Node> SIPP::get_endpoints(int node_id, double node_i, double node_j, double t1, double t2)
{
    std::vector<Node> nodes;
    nodes = {Node(node_id, 0, 0, node_i, node_j, nullptr, t1, t2)};
    if(collision_intervals[node_id].empty())
        return nodes;
    else
        for(unsigned int k = 0; k < collision_intervals[node_id].size(); k++)
        {    
            unsigned int xloc(0);
            while(xloc < nodes.size())
            {
                Node n = nodes[xloc];
                auto c = collision_intervals[node_id][k];
                bool changed = false;
                if(c.first - CN_EPSILON < n.interval.first && c.second + CN_EPSILON > n.interval.second)
                {
                    nodes.erase(nodes.begin() + xloc);
                    changed = true;
                }
                else if(c.first - CN_EPSILON < n.interval.first && c.second > n.interval.first)
                {
                    nodes[xloc].interval.first = c.second;
                    changed = true;
                }
                else if(c.first - CN_EPSILON > n.interval.first && c.second + CN_EPSILON < n.interval.second)
                {
                    nodes[xloc].interval.second = c.first;
                    nodes.insert(nodes.begin() + xloc + 1, Node(node_id, 0, 0, node_i, node_j, nullptr, c.second, n.interval.second));
                    changed = true;
                }
                else if(c.first < n.interval.second && c.second + CN_EPSILON > n.interval.second)
                {
                    nodes[xloc].interval.second = c.first;
                    changed = true;
                }
                if(changed)
                {
                    xloc = -1;
                    k = 0;
                }
                xloc++;
            }
        }
    return nodes;
}

double SIPP::check_endpoint(Node start, Node goal)
{
    double cost = sqrt(pow(start.xloc - goal.xloc, 2) + pow(start.yloc - goal.yloc, 2));
    if(start.g + cost < goal.interval.first)
        start.g = goal.interval.first - cost;
    if(constraints.count({start.id, goal.id}) != 0)
    {
        auto it = constraints.find({start.id, goal.id});
        for(unsigned int xloc = 0; xloc < it->second.size(); xloc++)
            if(start.g + CN_EPSILON > it->second[xloc].t1 && start.g < it->second[xloc].t2)
                start.g = it->second[xloc].t2;
    }
    if(start.g > start.interval.second || start.g + cost > goal.interval.second)
        return CN_INFINITY;
    else
        return start.g + cost;
}

Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic &h_values)
{
    this->clear();
    this->agent = agent;
    make_constraints(cons);

    std::vector<Node> starts, goals;
    std::vector<Path> parts, results, new_results;
    Path part, result;
    int expanded(0);
    if(!landmarks.empty())
    {
        for(unsigned int xloc = 0; xloc <= landmarks.size(); xloc++)
        {
            if(xloc == 0)
            {
                starts = {get_endpoints(agent.start_id, agent.start_xloc, agent.start_yloc, 0, CN_INFINITY).at(0)};
                goals = get_endpoints(landmarks[xloc].id1, map.get_i(landmarks[xloc].id1), map.get_j(landmarks[xloc].id1), landmarks[xloc].t1, landmarks[xloc].t2);
            }
            else
            {
                starts.clear();
                for(auto p:results)
                    starts.push_back(p.nodes.back());
                if(xloc == landmarks.size())
                    goals = {get_endpoints(agent.goal_id, agent.goal_xloc, agent.goal_yloc, 0, CN_INFINITY).back()};
                else
                    goals = get_endpoints(landmarks[xloc].id1, map.get_i(landmarks[xloc].id1), map.get_j(landmarks[xloc].id1), landmarks[xloc].t1, landmarks[xloc].t2);
            }
            if(goals.empty())
                return Path();
            parts = find_partial_path(starts, goals, map, h_values, goals.back().interval.second);
            expanded += int(close.size());
            new_results.clear();
            if(xloc == 0)
                for(unsigned int k = 0; k < parts.size(); k++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    new_results.push_back(parts[k]);
                }
            for(unsigned int k = 0; k < parts.size(); k++)
                for(unsigned int yloc = 0; yloc < results.size(); yloc++)
                {
                    if(parts[k].nodes.empty())
                        continue;
                    if(fabs(parts[k].nodes[0].interval.first - results[yloc].nodes.back().interval.first) < CN_EPSILON && fabs(parts[k].nodes[0].interval.second - results[yloc].nodes.back().interval.second) < CN_EPSILON)
                    {
                        new_results.push_back(results[yloc]);
                        new_results.back() = add_part(new_results.back(), parts[k]);
                    }
                }
            results = new_results;
            if(results.empty())
                return Path();
            if(xloc < landmarks.size())
            {
                starts.clear();
                for(auto p:results)
                    starts.push_back(p.nodes.back());
                double offset = sqrt(pow(map.get_i(landmarks[xloc].id1) - map.get_i(landmarks[xloc].id2), 2) + pow(map.get_j(landmarks[xloc].id1) - map.get_j(landmarks[xloc].id2), 2));
                goals = get_endpoints(landmarks[xloc].id2, map.get_i(landmarks[xloc].id2), map.get_j(landmarks[xloc].id2), landmarks[xloc].t1 + offset, landmarks[xloc].t2 + offset);
                if(goals.empty())
                    return Path();
                new_results.clear();
                for(unsigned int k = 0; k < goals.size(); k++)
                {
                    double best_g(CN_INFINITY);
                    int best_start_id = -1;
                    for(unsigned int yloc = 0; yloc < starts.size(); yloc++)
                    {
                        double g = check_endpoint(starts[yloc], goals[k]);
                        if(g < best_g)
                        {
                            best_start_id = yloc;
                            best_g = g;
                        }
                    }
                    if(best_start_id >= 0)
                    {
                        goals[k].g = best_g;
                        if(collision_intervals[goals[k].id].empty())
                            goals[k].interval.second = CN_INFINITY;
                        else
                        {
                            for(auto c:collision_intervals[goals[k].id])
                                if(goals[k].g < c.first)
                                {
                                    goals[k].interval.second = c.first;
                                    break;
                                }
                        }
                        new_results.push_back(results[best_start_id]);
                        if(goals[k].g - starts[best_start_id].g > offset + CN_EPSILON)
                        {
                            new_results.back().nodes.push_back(new_results.back().nodes.back());
                            new_results.back().nodes.back().g = goals[k].g - offset;
                        }
                        new_results.back().nodes.push_back(goals[k]);
                    }
                }

                results = new_results;
                if(results.empty())
                    return Path();
            }
        }
        result = results[0];
    }
    else
    {
        starts = {get_endpoints(agent.start_id, agent.start_xloc, agent.start_yloc, 0, CN_INFINITY).at(0)};
        goals = {get_endpoints(agent.goal_id, agent.goal_xloc, agent.goal_yloc, 0, CN_INFINITY).back()};
        parts = find_partial_path(starts, goals, map, h_values);
        expanded = int(close.size());
        if(parts[0].cost < 0)
            return Path();
        result = parts[0];
    }
    result.nodes.shrink_to_fit();
    result.cost = result.nodes.back().g;
    result.agentID = agent.id;
    result.expanded = expanded;
    return result;
}
