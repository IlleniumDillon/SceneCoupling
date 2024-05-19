#include "Visualize.hpp"

void visualizeScene(Scene &scene, double res, const std::string &windowName)
{
    OccupancyGridMap map;
    scene.generateMap(map, res);
    cv::Mat img = cv::Mat::zeros(map.indexHeight, map.indexWidth, CV_8UC3);
    for (int i = 0; i < map.indexHeight; i++)
    {
        for (int j = 0; j < map.indexWidth; j++)
        {
            if (map(j, i) == 0)
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
            else
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    for (int i = 0; i < scene.uvObjects.size(); i++)
    {
        UVObject uvObject = scene.uvObjects[i];
        cv::circle(img, cv::Point(uvObject.position.x() / map.resolution, uvObject.position.y() / map.resolution), 5, cv::Scalar(0, 0, 255), -1);
    }
    cv::imshow(windowName, img);
    cv::waitKey(0);
}

void visualizeMap(OccupancyGridMap &map, const std::string &windowName)
{
    cv::Mat img = cv::Mat::zeros(map.indexHeight, map.indexWidth, CV_8UC3);
    for (int i = 0; i < map.indexHeight; i++)
    {
        for (int j = 0; j < map.indexWidth; j++)
        {
            if (map(j, i) == 0)
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
            else
            {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    cv::imshow(windowName, img);
    cv::waitKey(0);
}

void visualizeMap(VoronoiMap &map, const std::string &windowName)
{

}

void visualizePath(Path &path, const std::string &windowName)
{

}
