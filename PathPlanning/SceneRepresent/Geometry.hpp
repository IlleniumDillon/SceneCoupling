#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include "eigen3/Eigen/Eigen"

bool checkIntersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2);
bool checkCollinear(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3);
bool checkInPolygon(Eigen::Vector2d point, std::vector<Eigen::Vector2d>& polygon);
bool checkInCircle(Eigen::Vector2d point, Eigen::Vector2d center, double radius);

void drawLine(uint8_t** gridMap, int width, int height, Eigen::Vector2i p1, Eigen::Vector2i p2, uint8_t value);

/// @brief CoordinateConverter class
/// @details This class is a singleton class that provides methods 
///          to convert coordinates between different coordinate systems
/// @note Scene coordinate system: 
///       - Origin at middle of the scene
///       - x-axis points to right
///       - y-axis points to up
/// @note Render & GridMap coordinate system:
///       - Origin at top-left corner
///       - x-axis points to right
///       - y-axis points to down
/// @note Agent coordinate system:
///       - Origin at middle of the agent
///       - x-axis points to up
///       - y-axis points to left
///       - position is relative to the agent
///       - rotation of the agent is considered, relative to the scene coordinate system
class CoordinateConverter
{
private:
    CoordinateConverter(){}
    CoordinateConverter(const CoordinateConverter &) = delete;
    CoordinateConverter(const CoordinateConverter &&) = delete;
    CoordinateConverter &operator=(const CoordinateConverter &) = delete;
public:
    static CoordinateConverter &getInstance()
    {
        static CoordinateConverter instance;
        return instance;
    }
    void init(
        double sceneWidth, 
        double sceneHeight, 
        double mapResolution, 
        double renderResolution
    );
    void cvtSceneToRender(Eigen::Vector2d &i, Eigen::Vector2i &o);
    void cvtRenderToScene(Eigen::Vector2i &i, Eigen::Vector2d &o);
    void cvtSceneToGridmap(Eigen::Vector2d &i, Eigen::Vector2i &o);
    void cvtGridmapToScene(Eigen::Vector2i &i, Eigen::Vector2d &o);
    void cvtSceneToAgent(Eigen::Vector2d &agent, double yaw, Eigen::Vector2d &i, Eigen::Vector2d &o);
    void cvtAgentToScene(Eigen::Vector2d &agent, double yaw, Eigen::Vector2d &i, Eigen::Vector2d &o);
    void cvtAgentToGridmap(Eigen::Vector2d &agent, double yaw, Eigen::Vector2d &i, Eigen::Vector2i &o);
    void cvtGridmapToAgent(Eigen::Vector2d &agent, double yaw, Eigen::Vector2i &i, Eigen::Vector2d &o);
    void cvtAgentToRender(Eigen::Vector2d &agent, double yaw, Eigen::Vector2d &i, Eigen::Vector2i &o);
    void cvtRenderToAgent(Eigen::Vector2d &agent, double yaw, Eigen::Vector2i &i, Eigen::Vector2d &o);
    void cvtGridmapToRender(Eigen::Vector2i &i, Eigen::Vector2i &o);
    void cvtRenderToGridmap(Eigen::Vector2i &i, Eigen::Vector2i &o);
private:
    double sceneWidth;
    double sceneHeight;
    double mapResolution;
    double renderResolution;
    int renderWidth;
    int renderHeight;
    int gridmapWidth;
    int gridmapHeight;
    Eigen::Vector2i sceneOriginToRender;
    Eigen::Vector2d renderOriginToScene;
    Eigen::Vector2i sceneOriginToGridmap;
    Eigen::Vector2d gridmapOriginToScene;
};

#endif // GEOMETRY_HPP