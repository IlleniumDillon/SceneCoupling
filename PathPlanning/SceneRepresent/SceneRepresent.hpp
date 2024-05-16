#ifndef SCENEREPRESENT_HPP
#define SCENEREPRESENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Eigen"

#include "MapRepresent.hpp"

/// todo: add actions
class SceneObject
{
public:
    SceneObject(std::string name);
    SceneObject(std::string name, const std::vector<Eigen::Vector2d>& vertices, double orientation, const Eigen::Vector2d& position);

    void setVertices(const std::vector<Eigen::Vector2d>& vertices);
    void setOrientation(double orientation);
    void setPosition(const Eigen::Vector2d& position);

    void rotate(double angle);
    void translate(const Eigen::Vector2d& translation);

    void updateCurrentVertices();
public:
    std::string name;
    Eigen::Vector2d position; // the position of the centroid of the object
    double orientation;     // in radians
    std::vector<Eigen::Vector2d> vertices; // the vertices of the object(when oriented at 0 radians and positioned at (0,0))
    std::vector<Eigen::Vector2d> currentVertices; // the vertices of the object(when oriented at orientation and positioned at position)
};

class Scene
{
public:
    Scene();
    Scene(std::string filename);

    void setSceneSize(double width, double height);
    void setOrigin(double x, double y);

    void addObject(const SceneObject& object);
    void removeObject(int index);

    void generateMap(OccupancyGridMap& map, double resolution);
public:
    std::vector<SceneObject> objects;
    double width;
    double height;
    Eigen::Vector2d origin;
};

#endif // SCENEREPRESENT_HPP