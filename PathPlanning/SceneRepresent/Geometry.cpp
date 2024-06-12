#include "Geometry.hpp"

double fragments = 0.001;

double crossProduct(Eigen::Vector2d a, Eigen::Vector2d b)
{
    return a.x() * b.y() - a.y() * b.x();
}

bool checkIntersect(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3, Eigen::Vector2d p4)
{
    // 快速排斥实验
    if (std::max(p1.x(), p2.x()) < std::min(p3.x(), p4.x()) ||
        std::max(p3.x(), p4.x()) < std::min(p1.x(), p2.x()) ||
        std::max(p1.y(), p2.y()) < std::min(p3.y(), p4.y()) ||
        std::max(p3.y(), p4.y()) < std::min(p1.y(), p2.y()))
    {
        return false;
    }
    // 跨立实验
    double u = (p3.x() - p1.x()) * (p2.y() - p1.y()) - (p2.x() - p1.x()) * (p3.y() - p1.y());
    double v = (p4.x() - p1.x()) * (p2.y() - p1.y()) - (p2.x() - p1.x()) * (p4.y() - p1.y());
    double w = (p1.x() - p3.x()) * (p4.y() - p3.y()) - (p4.x() - p3.x()) * (p1.y() - p3.y());
    double z = (p2.x() - p3.x()) * (p4.y() - p3.y()) - (p4.x() - p3.x()) * (p2.y() - p3.y());
    return u * v <= 0 && w * z <= 0;
}

bool checkCollinear(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3)
{
    Eigen::Matrix3d matrix;
    matrix << p1.x(), p1.y(), 1,
        p2.x(), p2.y(), 1,
        p3.x(), p3.y(), 1;
    return matrix.determinant() < fragments;
}

bool checkInPolygon(Eigen::Vector2d point, std::vector<Eigen::Vector2d> &polygon)
{
    int count = 0;
    for (int i = 0; i < polygon.size(); i++)
    {
        Eigen::Vector2d p1 = polygon[i];
        Eigen::Vector2d p2 = polygon[(i + 1) % polygon.size()];
        // check if point is on the edge
        if (checkCollinear(point, p1, p2))
        {
            return true;
        }
        
        if (checkIntersect(point, Eigen::Vector2d(point.x(), 10000), p1, p2))
        {
            count++;
        }
    }
    if (count > 2)
    {
        std::cout << "count: " << count << std::endl;
    }
    if (count % 2 == 1)
    {
        return true;
    }
    return false;
}

bool checkInCircle(Eigen::Vector2d point, Eigen::Vector2d center, double radius)
{
    double distance = (point - center).norm();
    if (distance <= radius)
    {
        return true;
    }
    return false;
}

void CoordinateConverter::init(double sceneWidth, double sceneHeight, double mapResolution, double renderResolution)
{
    this->sceneWidth = sceneWidth;
    this->sceneHeight = sceneHeight;
    this->mapResolution = mapResolution;
    this->renderResolution = renderResolution;
    renderWidth = sceneWidth / renderResolution;
    renderHeight = sceneHeight / renderResolution;
    gridmapWidth = sceneWidth / mapResolution;
    gridmapHeight = sceneHeight / mapResolution;
    sceneOriginToRender = Eigen::Vector2i(renderWidth / 2, renderHeight / 2);
    renderOriginToScene = Eigen::Vector2d(-sceneWidth / 2, sceneHeight / 2);
    sceneOriginToGridmap = Eigen::Vector2i(gridmapWidth / 2, gridmapHeight / 2);
    gridmapOriginToScene = Eigen::Vector2d(-sceneWidth / 2, sceneHeight / 2);
}

void CoordinateConverter::cvtSceneToRender(Eigen::Vector2d &i, Eigen::Vector2i &o)
{
    o.x() = sceneOriginToRender.x() + i.x() / renderResolution;
    o.y() = sceneOriginToRender.y() - i.y() / renderResolution;
}

void CoordinateConverter::cvtRenderToScene(Eigen::Vector2i &i, Eigen::Vector2d &o)
{
    o.x() = renderOriginToScene.x() + i.x() * renderResolution;
    o.y() = renderOriginToScene.y() - i.y() * renderResolution;
}

void CoordinateConverter::cvtSceneToGridmap(Eigen::Vector2d &i, Eigen::Vector2i &o)
{
    o.x() = sceneOriginToGridmap.x() + i.x() / mapResolution;
    o.y() = sceneOriginToGridmap.y() - i.y() / mapResolution;
}

void CoordinateConverter::cvtGridmapToScene(Eigen::Vector2i &i, Eigen::Vector2d &o)
{
    o.x() = gridmapOriginToScene.x() + i.x() * mapResolution;
    o.y() = gridmapOriginToScene.y() - i.y() * mapResolution;
}

/// TODO: CHECK THIS FUNCTION
void CoordinateConverter::cvtSceneToAgent(Eigen::Vector2d &agent, double yaw, Eigen::Vector2d &i, Eigen::Vector2d &o)
{
    
}

/// TODO: CHECK THIS FUNCTION
void CoordinateConverter::cvtAgentToScene(Eigen::Vector2d &agent, double yaw, Eigen::Vector2d &i, Eigen::Vector2d &o)
{
    
}

void CoordinateConverter::cvtAgentToGridmap(Eigen::Vector2d &agent, double yaw, Eigen::Vector2d &i, Eigen::Vector2i &o)
{
    Eigen::Vector2d pointAgentToScene;
    cvtAgentToScene(agent,yaw,i,pointAgentToScene);
    cvtSceneToGridmap(pointAgentToScene,o);
}

void CoordinateConverter::cvtGridmapToAgent(Eigen::Vector2d &agent, double yaw, Eigen::Vector2i &i, Eigen::Vector2d &o)
{
    Eigen::Vector2d pointGridmapToScene;
    cvtGridmapToScene(i,pointGridmapToScene);
    cvtSceneToAgent(agent,yaw,pointGridmapToScene,o);
}

void CoordinateConverter::cvtAgentToRender(Eigen::Vector2d &agent, double yaw, Eigen::Vector2d &i, Eigen::Vector2i &o)
{
    Eigen::Vector2d pointAgentToScene;
    cvtAgentToScene(agent,yaw,i,pointAgentToScene);
    cvtSceneToRender(pointAgentToScene,o);
}

void CoordinateConverter::cvtRenderToAgent(Eigen::Vector2d &agent, double yaw, Eigen::Vector2i &i, Eigen::Vector2d &o)
{
    Eigen::Vector2d pointRenderToScene;
    cvtRenderToScene(i,pointRenderToScene);
    cvtSceneToAgent(agent,yaw,pointRenderToScene,o);
}

void CoordinateConverter::cvtGridmapToRender(Eigen::Vector2i &i, Eigen::Vector2i &o)
{
    o.x() = i.x() * mapResolution / renderResolution;
    o.y() = i.y() * mapResolution / renderResolution;
}

void CoordinateConverter::cvtRenderToGridmap(Eigen::Vector2i &i, Eigen::Vector2i &o)
{
    o.x() = i.x() * renderResolution / mapResolution;
    o.y() = i.y() * renderResolution / mapResolution;
}
