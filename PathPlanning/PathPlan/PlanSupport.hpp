#ifndef PLAN_SUPPORT_HPP
#define PLAN_SUPPORT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <chrono>

#include "eigen3/Eigen/Eigen"
#include "opencv2/opencv.hpp"

#include "SceneRepresent.hpp"
#include "AgentRepresent.hpp"

enum SearchStatus : int8_t
{
    STATUS_OPEN,
    STATUS_CLOSE,
    STATUS_UNPROCESSED,
};

class GridNode
{
public:
    GridNode()
    {
        position = Eigen::Vector2d(0, 0);
        index = Eigen::Vector2i(0, 0);
        yaw = 0;
        gCost = std::numeric_limits<double>::max();
        hCost = 0;
        fCost = std::numeric_limits<double>::max();
        parent = nullptr;
        status = STATUS_UNPROCESSED;
        iterator = std::multimap<double, GridNode*>::iterator();
        isOccupied = false;
        whoOccupied = -1;
    }
    GridNode(Eigen::Vector2i index, bool isOccupied, int whoOccupied = -1)
    {
        this->index = index;
        this->position = Eigen::Vector2d(0, 0);
        this->yaw = 0;
        this->gCost = std::numeric_limits<double>::max();
        this->hCost = 0;
        this->fCost = std::numeric_limits<double>::max();
        this->parent = nullptr;
        this->status = STATUS_UNPROCESSED;
        this->iterator = std::multimap<double, GridNode*>::iterator();
        this->isOccupied = isOccupied;
        this->whoOccupied = whoOccupied;
    }
    bool equal(GridNode& node, double epsilon)
    {
        Eigen::Vector2d diff = position - node.position;
        return diff.norm() < epsilon;
    }
public:
    /// geometry
    Eigen::Vector2d position;
    Eigen::Vector2i index;
    double yaw;
    /// cost
    double gCost;
    double hCost;
    double fCost;
    /// parent
    GridNode* parent;
    /// search status
    SearchStatus status;
    /// piority queue iterator
    std::multimap<double, GridNode*>::iterator iterator;
    /// occupancy
    bool isOccupied;
    int whoOccupied;
};

class GridNodeMap
{
public:
    GridNodeMap()
    {
        width = 0;
        height = 0;
        resolution = 0;
        gridMap = nullptr;
    }
    GridNodeMap(int width, int height, double resolution)
    {
        this->width = width;
        this->height = height;
        this->resolution = resolution;
        gridMap = new GridNode*[width];
        for (int i = 0; i < width; i++)
        {
            gridMap[i] = new GridNode[height];
            for (int j = 0; j < height; j++)
            {
                gridMap[i][j].index = Eigen::Vector2i(i, j);
                CoordinateConverter::getInstance().cvtGridmapToScene(gridMap[i][j].index, gridMap[i][j].position);
            }
        }
    }
    GridNodeMap(GridNodeMap &map)
    {
        width = map.width;
        height = map.height;
        resolution = map.resolution;
        changedNodes = map.changedNodes;
        gridMap = new GridNode*[width];
        for (int i = 0; i < width; i++)
        {
            gridMap[i] = new GridNode[height];
            for (int j = 0; j < height; j++)
            {
                gridMap[i][j] = map.gridMap[i][j];
            }
        }
    }
    ~GridNodeMap()
    {
        for (int i = 0; i < width; i++)
        {
            delete[] gridMap[i];
        }
        delete[] gridMap;
        std::cout << "delete gridnode map" << std::endl;
    }
    GridNode& operator[](Eigen::Vector2i index)
    {
        assert(index.x() >= 0 && index.x() < width && index.y() >= 0 && index.y() < height);
        return gridMap[index.x()][index.y()];
    }
    GridNodeMap& operator=(GridNodeMap& map)
    {
        if (this == &map)
        {
            return *this;
        }
        width = map.width;
        height = map.height;
        resolution = map.resolution;
        changedNodes = map.changedNodes;
        if (gridMap != nullptr)
        {
            for (int i = 0; i < width; i++)
            {
                delete[] gridMap[i];
            }
            delete[] gridMap;
        }
        gridMap = new GridNode*[width];
        for (int i = 0; i < width; i++)
        {
            gridMap[i] = new GridNode[height];
            for (int j = 0; j < height; j++)
            {
                gridMap[i][j] = map.gridMap[i][j];
            }
        }
        return *this;
    }
    void init(int width, int height, double resolution)
    {
        this->width = width;
        this->height = height;
        this->resolution = resolution;
        gridMap = new GridNode*[width];
        for (int i = 0; i < width; i++)
        {
            gridMap[i] = new GridNode[height];
            for (int j = 0; j < height; j++)
            {
                gridMap[i][j].index = Eigen::Vector2i(i, j);
                CoordinateConverter::getInstance().cvtGridmapToScene(gridMap[i][j].index, gridMap[i][j].position);
            }
        }
    }
    void releaseChangedNodes()
    {
        for (auto node : changedNodes)
        {
            node->yaw = 0;
            node->gCost = std::numeric_limits<double>::max();
            node->hCost = 0;
            node->fCost = std::numeric_limits<double>::max();
            node->parent = nullptr;
            node->status = STATUS_UNPROCESSED;
            node->iterator = std::multimap<double, GridNode*>::iterator();
        }
    }
public:
    int width;
    int height;
    double resolution;
    GridNode** gridMap = nullptr;
    std::vector<GridNode*> changedNodes;
};

class PlanResult
{
public:
    PlanResult()
    {
        success = false;
        cost = -1;
        iteration = 0;
        time = 0;
    }
public:
    bool success;
    std::vector<AgentAction> actions;
    double cost;
    int iteration;
    double time;
};

#endif // PLAN_SUPPORT_HPP