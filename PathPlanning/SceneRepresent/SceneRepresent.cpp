#include "SceneRepresent.hpp"

SceneObject::SceneObject(std::string name) : name(name)
{
    position = Eigen::Vector2d(0, 0);
    orientation = 0;
    vertices.clear();
    currentVertices.clear();
}

SceneObject::SceneObject(std::string name, const std::vector<Eigen::Vector2d> &vertices, double orientation, const Eigen::Vector2d &position)
{
    this->name = name;
    this->vertices = vertices;
    this->orientation = orientation;
    this->position = position;
    currentVertices.clear();

    updateCurrentVertices();
}

void SceneObject::setVertices(const std::vector<Eigen::Vector2d> &vertices)
{
    this->vertices = vertices;
}

void SceneObject::setOrientation(double orientation)
{
    this->orientation = orientation;
}

void SceneObject::setPosition(const Eigen::Vector2d &position)
{
    this->position = position;
}

void SceneObject::rotate(double angle)
{
    this->orientation += angle;
}

void SceneObject::translate(const Eigen::Vector2d &translation)
{
    this->position += translation;
}

void SceneObject::updateCurrentVertices()
{
    currentVertices.clear();
    for (int i = 0; i < vertices.size(); i++)
    {
        Eigen::Vector2d vertex = vertices[i];
        Eigen::Vector2d rotatedVertex = Eigen::Rotation2Dd(orientation) * vertex;
        currentVertices.push_back(rotatedVertex + position);
    }
}

Scene::Scene()
{
    this->height = 0;
    this->width = 0;
    this->origin = Eigen::Vector2d(0, 0);
    objects.clear();
}

Scene::Scene(std::string filename)
{
}

void Scene::setSceneSize(double width, double height)
{
    this->width = width;
    this->height = height;
}

void Scene::setOrigin(double x, double y)
{
    this->origin = Eigen::Vector2d(x, y);
}

void Scene::addObject(const SceneObject &object)
{
    objects.push_back(object);
}

void Scene::removeObject(int index)
{
    objects.erase(objects.begin() + index);
}

void Scene::generateMap(OccupancyGridMap &map, double resolution = 0.05)
{
    map.setMapSize(width, height, resolution);
    map.setOrigin(origin.x(), origin.y());
    for (int i = 0; i < objects.size(); i++)
    {
        SceneObject object = objects[i];
        // polygonize the object
        map.polygon(object.currentVertices);
    }
}
