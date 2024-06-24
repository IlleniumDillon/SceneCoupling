#include <iostream>
#include "testmain.hpp"

int main() 
{
    Scene scene(SCENE_FILE_PATH);
    Agent agent("agent1", &scene);
    // agent.attachObject("box17", 
    //                 Eigen::Vector3d(scene["agent1"]->center.x(), scene["agent1"]->center.y(), scene["agent1"]->rotation), 
    //                 Eigen::Vector3d(scene["box17"]->center.x(), scene["box17"]->center.y(), scene["box17"]->rotation));
    //std::cout << agent.relativeTransforms[0] << std::endl;
    SimpleTransferTask task("agent1", {-2, -2}, 0);
    Planner planner;
    planner.configure(agent);

    PlanResult rst = planner.plan(task);

    std::cout << "time: " << rst.time << std::endl;
    std::cout << "actions: " << rst.actions.size() << std::endl;
    std::cout << "cost: " << rst.cost << std::endl;
    std::cout << "iteration: " << rst.iteration << std::endl;

    visualize(scene, agent, rst, 1);

    return 0;
}