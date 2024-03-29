#include <iostream>
#include <fstream>
#include "map.h"
#include "task.h"
#include "cbs.h"
#include "xml_logger.h"

int main(int argc, const char *argv[])
{
    /* config content
        <use_cardinal>true</use_cardinal>
		<use_disjoint_splitting>true</use_disjoint_splitting>
		<hlh_type>2</hlh_type>
		<connectedness>3</connectedness>
		<focal_weight>1.0</focal_weight>
		<agent_size>0.5</agent_size>
		<timelimit>30</timelimit>
		<precision>0.0000001</precision>
    */
    const std::string configFile = "Examples/config.xml";
    const std::string mapFile = "Examples/roadmap.xml";
    const std::string taskFile = "Examples/roadmap_task.xml";
    Config config;
    config.getConfig(configFile.c_str());
    Map map = Map(config.agent_size, config.connectdness);
    map.get_roadmap(mapFile.c_str());
    Task task;
    task.get_task(taskFile.c_str(),5);
    task.make_ij(map);

    CBS cbs;
    Solution solution = cbs.find_solution(map, task, config);
    XML_logger logger;
    auto found = solution.found?"true":"false";
    std::cout<< "Soulution found: " << found << "\nRuntime: "<<solution.time.count() << "\nMakespan: " << solution.makespan << "\nFlowtime: " << solution.flowtime<< "\nInitial Cost: "<<solution.init_cost<< "\nCollision Checking Time: " << solution.check_time
            << "\nHL expanded: " << solution.high_level_expanded << "\nLL searches: " << solution.low_level_expansions << "\nLL expanded(avg): " << solution.low_level_expanded << std::endl;

    logger.get_log(taskFile.c_str());
    logger.write_to_log_summary(solution);
    logger.write_to_log_path(solution, map);
    logger.save_log();

    return 0;
}
