#include <iostream>
#include "testmain.hpp"

int main() 
{
    Agent agent("agent", SCENE_FILE_PATH);
    SimpleTransferTask task("agent", {-2, -2}, 0);
    PlanResult rst = PathPlan(agent, task);

    std::cout << "time: " << rst.time << std::endl;

    agent.scene.renderScene();
    cv::Mat img = agent.scene.sceneImage;

    for (int i = 0; i < rst.actions.size(); i++)
    {
        // std::cout << rst.actions[i].position << std::endl;
        Eigen::Vector2i pointGridmap;
        CoordinateConverter::getInstance().cvtSceneToRender(rst.actions[i].position, pointGridmap);
        cv::circle(img, cv::Point(pointGridmap.x(), pointGridmap.y()), 2, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("Path Plan", img);
    cv::waitKey(0);

    return 0;
}