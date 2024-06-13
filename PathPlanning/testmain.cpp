#include <iostream>
#include "testmain.hpp"

int main() 
{
    Scene scene(SCENE_FILE_PATH);
    Agent agent("agent1", &scene);
    SimpleTransferTask task("agent1", {-2, -2}, 0);
    Planner planner;
    planner.configure(agent);
    PlanResult rst = planner.plan(task);

    std::cout << "time: " << rst.time << std::endl;

    visualize(scene, agent, rst, 10);

    return 0;
}