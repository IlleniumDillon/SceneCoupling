#include "SceneRepresent.hpp"


SceneObject::SceneObject()
{
    this->name = "";
    this->type = "";
    this->action = "";
    this->center = Eigen::Vector2d(0, 0);
    this->rotation = 0;
    this->vertices.clear();
    this->verticesTransformed.clear();
    this->anchor.clear();
    this->anchorTransformed.clear();
    this->needTransform = false;
}

SceneObject::SceneObject(std::string name, std::string type, std::string action, Eigen::Vector2d center, double rotation, std::vector<Eigen::Vector2d>& vertices, std::vector<Eigen::Vector2d>& anchor)
{
    this->name = name;
    this->type = type;
    this->action = action;
    this->center = center;
    this->rotation = rotation;
    this->vertices = vertices;
    this->anchor = anchor;
    this->needTransform = true;

    updateGeometry();
}

SceneObject::~SceneObject()
{
}

void SceneObject::updateGeometry()
{
    if (needTransform)
    {
        Eigen::Matrix2d rotationMatrix;
        while (rotation >= 2 * M_PI || rotation < 0)
        {
            if (rotation >= 2 * M_PI)
            {
                rotation -= 2 * M_PI;
            }
            else if (rotation < 0)
            {
                rotation += 2 * M_PI;
            }
        }
       
        rotationMatrix << cos(rotation), -sin(rotation), sin(rotation), cos(rotation);

        verticesTransformed.clear();
        for (int i = 0; i < vertices.size(); i++)
        {
            Eigen::Vector2d vertex = vertices[i];
            Eigen::Vector2d vertexTransformed = rotationMatrix * vertex + center;
            verticesTransformed.push_back(vertexTransformed);
        }

        anchorTransformed.clear();
        for (int i = 0; i < anchor.size(); i++)
        {
            Eigen::Vector2d anchorPoint = anchor[i];
            Eigen::Vector2d anchorPointTransformed = rotationMatrix * anchorPoint + center;
            anchorTransformed.push_back(anchorPointTransformed);
        }

        needTransform = false;
    }
}

bool SceneObject::containPoint(Eigen::Vector2d point)
{
    if (type == "Polygon")
    {
        return checkInPolygon(point, verticesTransformed);
    }
    else if (type == "Circle")
    {
        return checkInCircle(point, center, (verticesTransformed[0] - center).norm());
    }
    return false;
}

void SceneObject::moveObject(Eigen::Vector2d translation)
{
    center += translation;
    needTransform = true;
}

void SceneObject::rotateObject(double angle)
{
    rotation += angle;
    needTransform = true;
}

SceneObject &SceneObject::operator=(const SceneObject &object)
{
    this->name = object.name;
    this->type = object.type;
    this->action = object.action;
    this->center = object.center;
    this->rotation = object.rotation;
    this->vertices = object.vertices;
    this->verticesTransformed = object.verticesTransformed;
    this->anchor = object.anchor;
    this->anchorTransformed = object.anchorTransformed;
    this->needTransform = true;
    return *this;
}

Scene::Scene()
{
    this->width = 0;
    this->height = 0;
    this->resolution = 0;
    this->sceneObjects.clear();
    this->agentObjects.clear();
    this->sceneImage = cv::Mat();
}

Scene::Scene(std::string scnFilePath)
{
    std::ifstream scnFile(scnFilePath);
    if (!scnFile.is_open())
    {
        std::cerr << "Error: Cannot open scene file." << std::endl;
        return;
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(scnFile, root))
    {
        std::cerr << "Error: Cannot parse scene file." << std::endl;
        return;
    }

    scnFile.close();

    if (root["width"].isNull() || 
        root["height"].isNull() || 
        root["resolution"].isNull() ||
        root["primitives"].isNull() ||
        root["agents"].isNull())
    {
        std::cerr << "Error: Scene file is invalid." << std::endl;
        return;
    }

    this->width = root["width"].asDouble();
    this->height = root["height"].asDouble();
    this->resolution = root["resolution"].asDouble();
    Json::Value agents = root["agents"];

    Json::Value primitives = root["primitives"];
    for (int i = 0; i < primitives.size(); i++)
    {
        Json::Value primitive = primitives[i];
        if (primitive["name"].isNull() || 
            primitive["type"].isNull() || 
            primitive["center"].isNull() || 
            primitive["rotation"].isNull() || 
            primitive["vertices"].isNull() || 
            primitive["action"].isNull() ||
            (!primitive["anchor"].isNull() && primitive["anchor"].isNull()))
        {
            std::cerr << "Error: Primitive is invalid." << std::endl;
            continue;
        }

        std::string name = primitive["name"].asString();
        std::string type = primitive["type"].asString();
        Eigen::Vector2d center(primitive["center"][0].asDouble(), primitive["center"][1].asDouble());
        double rotation = primitive["rotation"].asDouble();
        std::vector<Eigen::Vector2d> vertices;
        for (int j = 0; j < primitive["vertices"].size(); j++)
        {
            Eigen::Vector2d vertex(primitive["vertices"][j][0].asDouble(), primitive["vertices"][j][1].asDouble());
            vertices.push_back(vertex);
        }
        std::string action = primitive["action"].asString();
        std::vector<Eigen::Vector2d> anchor;
        if (action != "None")
        {
            for (int j = 0; j < primitive["anchor"].size(); j++)
            {
                Eigen::Vector2d anchorPoint(primitive["anchor"][j][0].asDouble(), primitive["anchor"][j][1].asDouble());
                anchor.push_back(anchorPoint);
            }
        }

        SceneObject sceneObject(name, type, action, center, rotation, vertices, anchor);

        // bool isAgent = false;
        // for (int j = 0; j < agents.size(); j++)
        // {
        //     std::string agentName = agents[j].asString();
        //     if (agentName == name)
        //     {
        //         isAgent = true;
        //         break;
        //     }
        // }

        // if (isAgent)
        // {
        //     agentObjects.push_back(sceneObject);
        // }
        // else
        // {
        //     sceneObjects.push_back(sceneObject);
        // }
        sceneObjects.push_back(sceneObject);
    }

    int targetCols = 960;
    int targetRows = 540;

    renderResolution = std::min(width / targetCols, height / targetRows);

    CoordinateConverter::getInstance().init(width, height, resolution, renderResolution);
}

Scene::~Scene()
{
}

void Scene::update()
{
    for (int i = 0; i < sceneObjects.size(); i++)
    {
        sceneObjects.at(i).updateGeometry();
    }
}

void Scene::renderScene()
{
    sceneImage = cv::Mat(height / renderResolution, width / renderResolution, CV_8UC3, cv::Scalar(255,255,255));

    for (int i = 0; i < sceneObjects.size(); i++)
    {
        if (sceneObjects.at(i).type == "Polygon")
        {
            std::vector<cv::Point> polygon;
            for (int j = 0; j < sceneObjects.at(i).verticesTransformed.size(); j++)
            {
                Eigen::Vector2d vertex = sceneObjects.at(i).verticesTransformed[j];
                Eigen::Vector2i vertexRender;
                CoordinateConverter::getInstance().cvtSceneToRender(vertex, vertexRender);
                polygon.push_back(cv::Point(vertexRender.x(), vertexRender.y()));
            }
            cv::fillConvexPoly(sceneImage, polygon.data(), polygon.size(), cv::Scalar(0, 0, 0));
        }
        else if (sceneObjects.at(i).type == "Circle")
        {
            Eigen::Vector2d center = sceneObjects.at(i).center;
            Eigen::Vector2i centerRender;
            CoordinateConverter::getInstance().cvtSceneToRender(center, centerRender);
            int radius = (sceneObjects.at(i).verticesTransformed[0] - center).norm() / renderResolution;
            cv::circle(sceneImage, cv::Point(centerRender.x(), centerRender.y()), radius, cv::Scalar(0, 0, 0), -1);
        }
    }

    for (int i = 0; i < agentObjects.size(); i++)
    {
        if (agentObjects.at(i).type == "Polygon")
        {
            std::vector<cv::Point> polygon;
            for (int j = 0; j < agentObjects.at(i).verticesTransformed.size(); j++)
            {
                Eigen::Vector2d vertex = agentObjects.at(i).verticesTransformed[j];
                Eigen::Vector2i vertexRender;
                CoordinateConverter::getInstance().cvtSceneToRender(vertex, vertexRender);
                polygon.push_back(cv::Point(vertexRender.x(), vertexRender.y()));
            }
            cv::fillConvexPoly(sceneImage, polygon.data(), polygon.size(), cv::Scalar(0, 255, 0));
        }
        else if (agentObjects.at(i).type == "Circle")
        {
            Eigen::Vector2d center = agentObjects.at(i).center;
            Eigen::Vector2i centerRender;
            CoordinateConverter::getInstance().cvtSceneToRender(center, centerRender);
            int radius = (agentObjects.at(i).verticesTransformed[0] - center).norm() / renderResolution;
            cv::circle(sceneImage, cv::Point(centerRender.x(), centerRender.y()), radius, cv::Scalar(0, 255, 0), -1);
        }
    }

    // cv::imshow("Scene", sceneImage);
    // cv::waitKey(0);
}

void Scene::toGridMap(GridMap &gridMap)
{
    int gridmapWidth = width / resolution;
    int gridmapHeight = height / resolution;

    gridMap = GridMap(gridmapWidth, gridmapHeight, resolution);

    for (int i = 0; i < sceneObjects.size(); i++)
    {
        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double maxX = -std::numeric_limits<double>::max();
        double maxY = -std::numeric_limits<double>::max();
        for (int j = 0; j < sceneObjects.at(i).verticesTransformed.size(); j++)
        {
            double x = sceneObjects.at(i).verticesTransformed[j].x();
            double y = sceneObjects.at(i).verticesTransformed[j].y();
            minX = std::min(minX, x);
            minY = std::min(minY, y);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
        }
        for (double x = minX; x <= maxX; x += resolution)
        {
            for (double y = minY; y <= maxY; y += resolution)
            {
                Eigen::Vector2d point(x, y);
                Eigen::Vector2i pointGridmap;
                CoordinateConverter::getInstance().cvtSceneToGridmap(point, pointGridmap);
                if (sceneObjects.at(i).containPoint(point) && 
                    pointGridmap.x() >= 0 && pointGridmap.x() < gridmapWidth && 
                    pointGridmap.y() >= 0 && pointGridmap.y() < gridmapHeight)
                {
                    gridMap(pointGridmap.x(), pointGridmap.y()) = 1;
                }
            }
        }
    }
}

Scene &Scene::operator=(const Scene &scene)
{
    this->width = scene.width;
    this->height = scene.height;
    this->resolution = scene.resolution;
    this->renderResolution = scene.renderResolution;
    this->sceneObjects = scene.sceneObjects;
    this->agentObjects = scene.agentObjects;
    this->sceneImage = scene.sceneImage;
    return *this;
}

SceneObject* Scene::operator[](int index)
{
    if (index < 0 || index >= sceneObjects.size())
    {
        return nullptr;
    }
    return &sceneObjects[index];
}

SceneObject* Scene::operator[](std::string name)
{
    for (int i = 0; i < sceneObjects.size(); i++)
    {
        if (sceneObjects[i].name == name)
        {
            return &sceneObjects[i];
        }
    }
    return nullptr;
}
