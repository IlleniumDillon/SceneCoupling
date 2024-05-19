#include "SceneRepresent.hpp"
#include "json/json.h"

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
}

Scene::Scene(std::string filename)
{
    Json::Value sceneValue;
    Json::Reader reader;
    std::ifstream file(filename);
    if (!reader.parse(file, sceneValue))
    {
        std::cerr << "Failed to parse scene file: " << filename << std::endl
                  << reader.getFormattedErrorMessages();
        return;
    }

    Json::Value geometryValue = sceneValue["Geometry"];
    if (geometryValue.isNull())
    {
        std::cerr << "Failed to parse geometry in scene file: " << filename << std::endl;
        return;
    }
    if (geometryValue["Size"].isNull() || 
        geometryValue["Origin"].isNull())
    {
        std::cerr << "Failed to parse width or height in scene file: " << filename << std::endl;
        return;
    }
    this->origin = Eigen::Vector2d(geometryValue["Origin"]["X"].asDouble(), geometryValue["Origin"]["Y"].asDouble());
    this->width = geometryValue["Size"]["Width"].asDouble();
    this->height = geometryValue["Size"]["Height"].asDouble();

    Json::Value objectsValue = sceneValue["Objects"];
    if (objectsValue.isNull())
    {
        std::cerr << "Failed to parse objects in scene file: " << filename << std::endl;
        return;
    }
    objects.clear();
    for (int i = 0; i < objectsValue.size(); i++)
    {
        Json::Value objectValue = objectsValue[i];
        if (objectValue["Name"].isNull() || 
            objectValue["Vertices"].isNull() || 
            objectValue["Orientation"].isNull() || 
            objectValue["Position"].isNull())
        {
            std::cerr << "Failed to parse object in scene file: " << filename << std::endl;
            return;
        }
        std::string name = objectValue["Name"].asString();
        std::vector<Eigen::Vector2d> vertices;
        for (int j = 0; j < objectValue["Vertices"].size(); j++)
        {
            Eigen::Vector2d vertex(objectValue["Vertices"][j]["X"].asDouble(), objectValue["Vertices"][j]["Y"].asDouble());
            vertices.push_back(vertex);
        }
        double orientation = objectValue["Orientation"].asDouble();
        Eigen::Vector2d position(objectValue["Position"]["X"].asDouble(), objectValue["Position"]["Y"].asDouble());
        SceneObject object(name, vertices, orientation, position);
        objects.push_back(object);
    }

    Json::Value uvObjectsValue = sceneValue["UVObjects"];
    if (uvObjectsValue.isNull())
    {
        std::cerr << "Failed to parse uv objects in scene file: " << filename << std::endl;
        return;
    }
    uvObjects.clear();
    for (int i = 0; i < uvObjectsValue.size(); i++)
    {
        Json::Value uvObjectValue = uvObjectsValue[i];
        if (uvObjectValue["Name"].isNull() || 
            uvObjectValue["Position"].isNull() || 
            uvObjectValue["Orientation"].isNull())
        {
            std::cerr << "Failed to parse uv object in scene file: " << filename << std::endl;
            return;
        }
        std::string name = uvObjectValue["Name"].asString();
        Eigen::Vector2d position(uvObjectValue["Position"]["X"].asDouble(), uvObjectValue["Position"]["Y"].asDouble());
        double orientation = uvObjectValue["Orientation"].asDouble();
        UVObject uvObject(name, position, orientation);
        uvObjects.push_back(uvObject);
    }
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

UVObject::UVObject(std::string name)
{
    this->name = name;
    position = Eigen::Vector2d(0, 0);
    orientation = 0;
}

UVObject::UVObject(std::string name, const Eigen::Vector2d &position, double orientation)
{
    this->name = name;
    this->position = position;
    this->orientation = orientation;
}

void UVObject::actionMove(const Eigen::Vector2d &translation)
{
    position += translation;
}

void UVObject::actionMoveObj(const Eigen::Vector2d &translation, SceneObject &object)
{
    position += translation;
    object.translate(translation);
}