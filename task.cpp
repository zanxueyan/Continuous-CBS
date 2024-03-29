#include "task.h"
Task::Task()
{
    agents.clear();
}

bool Task::get_task(const char *FileName, int k)
{
    tinyxml2::XMLElement *root = 0, *agent = 0;
    tinyxml2::XMLDocument doc;

    // Load XML File
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS)
    {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }

    // Get ROOT element
    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root)
    {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' tag found in XML file!" << std::endl;
        return false;
    }

    for (agent = root->FirstChildElement(); agent; agent = agent->NextSiblingElement())
    {
        Agent a;
        a.start_xloc = agent->DoubleAttribute(CNS_TAG_START_I);
        a.start_yloc = agent->DoubleAttribute(CNS_TAG_START_J);
        a.start_id = agent->IntAttribute(CNS_TAG_START_ID);
        a.goal_xloc = agent->DoubleAttribute(CNS_TAG_GOAL_I);
        a.goal_yloc = agent->DoubleAttribute(CNS_TAG_GOAL_J);
        a.goal_id = agent->IntAttribute(CNS_TAG_GOAL_ID);
        a.id = int(agents.size());
        agents.push_back(a);
        if(int(agents.size()) == k)
            break;
    }
    return true;
}

void Task::make_ids(int width)
{
    for(size_t xloc = 0; xloc < agents.size(); xloc++)
    {
        agents[xloc].start_id = int(agents[xloc].start_xloc)*width + int(agents[xloc].start_yloc);
        agents[xloc].goal_id = int(agents[xloc].goal_xloc)*width + int(agents[xloc].goal_yloc);
        //std::cout<<agents[i].start_i<<" "<<agents[i].start_j<<"  "<<agents[i].goal_i<<" "<<agents[i].goal_j<<"\n";
    }
}

void Task::make_ij(const Map& map)
{
    for(unsigned int xloc = 0; xloc < agents.size(); xloc++)
    {
        gNode start = map.get_gNode(agents[xloc].start_id), goal = map.get_gNode(agents[xloc].goal_id);
        agents[xloc].start_xloc = start.xloc;
        agents[xloc].start_yloc = start.yloc;
        agents[xloc].goal_xloc = goal.xloc;
        agents[xloc].goal_yloc = goal.yloc;
    }

}

Agent Task::get_agent(int id) const
{
    if(id >= 0 && id < int(agents.size()))
        return agents[id];
    else
        return Agent();
}
