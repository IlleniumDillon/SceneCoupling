#ifndef PATHPLAN_HPP
#define PATHPLAN_HPP

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <chrono>

#include "eigen3/Eigen/Eigen"
#include "opencv2/opencv.hpp"

#include "SceneRepresent.hpp"
#include "AgentRepresent.hpp"

#include "PlanSupport.hpp"

//PlanResult PathPlan(Agent& agent, SimpleTransferTask task);

class Planner
{
public:
    Planner(){}
    void configure(Agent agent);
    PlanResult plan(SimpleTransferTask task);
    PlanResult plan(MoveObjectTask task);
    void reset();   
public:
    Agent agent;
    std::vector<SceneObject> staticObjects;
    SceneObject agentObject;
    // std::vector<SceneObject> attachedObjects;
    // std::vector<Eigen::Matrix3d> relativeTransforms;
    // std::vector<Eigen::Vector2d> relativePositions;
    // std::vector<double> relativeRotations;

public:
    struct NodeInfo
    {
        double cost = 0;
        double yaw = 0;
    };
    double heuristic(GridNode* node, GridNode* goal);
    bool checkCollision(GridNode* node);
    bool __check(SceneObject& object);
    void getNeighbours(GridNode* current, std::vector<GridNode*>& neighbours, std::vector<NodeInfo>& neighboursInfo);

private:
    GridNodeMap gridMap;
    std::multimap<double, GridNode*> openSet;
};

#endif // PATHPLAN_HPP