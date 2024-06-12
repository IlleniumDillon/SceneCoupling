#ifndef SCENEREPRESENT_HPP
#define SCENEREPRESENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include "json/json.h"

#include "eigen3/Eigen/Eigen"
#include "opencv2/opencv.hpp"

#include "Geometry.hpp"
#include "GridMap.hpp"

class SceneObject
{
public:
    SceneObject();
    SceneObject(std::string name, std::string type, std::string action, Eigen::Vector2d center, double rotation, std::vector<Eigen::Vector2d>& vertices, std::vector<Eigen::Vector2d>& anchor);
    ~SceneObject();

    void updateGeometry();
    bool containPoint(Eigen::Vector2d point);

    void moveObject(Eigen::Vector2d translation);
    void rotateObject(double angle);

    SceneObject& operator=(const SceneObject &object);

public:
    std::string name;
    std::string type;
    std::string action;

    Eigen::Vector2d center;
    double rotation;

    std::vector<Eigen::Vector2d> vertices;
    std::vector<Eigen::Vector2d> verticesTransformed;

    std::vector<Eigen::Vector2d> anchor;
    std::vector<Eigen::Vector2d> anchorTransformed;

    bool needTransform;
};

class Scene
{
public:
    Scene();
    Scene(std::string scnFilePath);
    ~Scene();

    void update();

    void renderScene();
    void toGridMap(GridMap &gridMap);

    Scene& operator=(const Scene &scene);
    SceneObject* operator[](int index);
    SceneObject* operator[](std::string name);

public:
    double width;
    double height;
    double resolution;
    double renderResolution;

    std::vector<SceneObject> sceneObjects;
    std::vector<SceneObject> agentObjects;
    cv::Mat sceneImage;
};

#endif // SCENEREPRESENT_HPP