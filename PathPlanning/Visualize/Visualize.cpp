#include "Visualize.hpp"

void visualize(Scene scene, double fps)
{
    cv::Mat sceneImage = cv::Mat(scene.height / scene.renderResolution, scene.width / scene.renderResolution, CV_8UC3, cv::Scalar(255,255,255));

    for (int i = 0; i < scene.sceneObjects.size(); i++)
    {
        if (scene.sceneObjects.at(i).type == "Polygon")
        {
            std::vector<cv::Point> polygon;
            for (int j = 0; j < scene.sceneObjects.at(i).verticesTransformed.size(); j++)
            {
                Eigen::Vector2d vertex = scene.sceneObjects.at(i).verticesTransformed[j];
                Eigen::Vector2i vertexRender;
                CoordinateConverter::getInstance().cvtSceneToRender(vertex, vertexRender);
                polygon.push_back(cv::Point(vertexRender.x(), vertexRender.y()));
            }
            cv::fillConvexPoly(sceneImage, polygon.data(), polygon.size(), cv::Scalar(0, 0, 0));
        }
        else if (scene.sceneObjects.at(i).type == "Circle")
        {
            Eigen::Vector2d center = scene.sceneObjects.at(i).center;
            Eigen::Vector2i centerRender;
            CoordinateConverter::getInstance().cvtSceneToRender(center, centerRender);
            int radius = (scene.sceneObjects.at(i).verticesTransformed[0] - center).norm() / scene.renderResolution;
            cv::circle(sceneImage, cv::Point(centerRender.x(), centerRender.y()), radius, cv::Scalar(0, 0, 0), -1);
        }
    }

    cv::imshow("Visualize", sceneImage);
    cv::waitKey(1000 / fps);
}

void visualize(Scene scene, Agent agent, PlanResult result, double fps)
{
    cv::namedWindow("Visualize");

    visualize(scene ,fps);

    for (auto &action : result.actions)
    {
        if (action.type == AgentAction::ACTION_SIMPLE_TRANSFER)
        {
            scene[agent.name]->center = action.position;
            scene[agent.name]->rotation = action.yaw;
            scene[agent.name]->needTransform = true;
            scene[agent.name]->updateGeometry();

            // for (int i = 0; i < agent.attachedObjects.size(); i++)
            // {
            //     Eigen::Vector2d deltaPosition = {
            //         agent.relativePositions[i].x() * cos(action.yaw) - agent.relativePositions[i].y() * sin(action.yaw),
            //         agent.relativePositions[i].x() * sin(action.yaw) + agent.relativePositions[i].y() * cos(action.yaw) 
            //     };
            //     Eigen::Vector2d objectPosition = action.position + deltaPosition;
            //     double objectYaw = action.yaw + agent.relativeRotations[i];
            //     scene[agent.attachedObjects[i].name]->center = objectPosition;
            //     scene[agent.attachedObjects[i].name]->rotation = objectYaw;
            //     scene[agent.attachedObjects[i].name]->needTransform = true;
            //     scene[agent.attachedObjects[i].name]->updateGeometry();
            //     std::cout << "[" << objectPosition.x() << "," << objectPosition.y() << "," << objectYaw << "]" << std::endl;
            // }
        }


        visualize(scene, fps);
    }
}