#include "PathPlan.hpp"

// PlanResult PathPlan(Agent &agent, SimpleTransferTask task)
// {
//     PlanResult result;
//     /// geometry data
//     double minX = -agent.scene.width / 2;
//     double minY = -agent.scene.height / 2;
//     double maxX = agent.scene.width / 2;
//     double maxY = agent.scene.height / 2;
//     int gridWidth = agent.scene.width / agent.scene.resolution;
//     int gridHeight = agent.scene.height / agent.scene.resolution;
//     double gridResolution = agent.scene.resolution;

//     /// static objects are not movable
//     /// main objects are movable, are the target object and attached objects
//     std::vector<SceneObject> staticObjects;
//     std::vector<SceneObject> mainObjects;

//     for (auto &object: agent.scene.sceneObjects)
//     {
//         if (object.name != agent.name && 
//             std::find_if(agent.attachedObjects.begin(), agent.attachedObjects.end(), 
//                 [&object](SceneObject *attachedObject) { return object.name == attachedObject->name; }) == agent.attachedObjects.end())
//         {
//             staticObjects.push_back(object);
//         }
//         else
//         {
//             mainObjects.push_back(object);
//         }
//     }

//     /// create a grid map
//     GridNodeMap gridMap(gridWidth, gridHeight, gridResolution);
//     for (int i = 0; i < staticObjects.size(); i++)
//     {
//         double minX = std::numeric_limits<double>::max();
//         double minY = std::numeric_limits<double>::max();
//         double maxX = -std::numeric_limits<double>::max();
//         double maxY = -std::numeric_limits<double>::max();
//         for (int j = 0; j < staticObjects.at(i).verticesTransformed.size(); j++)
//         {
//             double x = staticObjects.at(i).verticesTransformed[j].x();
//             double y = staticObjects.at(i).verticesTransformed[j].y();
//             minX = std::min(minX, x);
//             minY = std::min(minY, y);
//             maxX = std::max(maxX, x);
//             maxY = std::max(maxY, y);
//         }
//         for (double x = minX; x <= maxX; x += gridResolution)
//         {
//             for (double y = minY; y <= maxY; y += gridResolution)
//             {
//                 Eigen::Vector2d point(x, y);
//                 Eigen::Vector2i pointGridmap;
//                 CoordinateConverter::getInstance().cvtSceneToGridmap(point, pointGridmap);
//                 if (staticObjects.at(i).containPoint(point) && 
//                     pointGridmap.x() >= 0 && pointGridmap.x() < gridWidth && 
//                     pointGridmap.y() >= 0 && pointGridmap.y() < gridHeight)
//                 {
//                     gridMap[pointGridmap].isOccupied = true;
//                 }
//             }
//         }
//     }

//     /// create a start node and a goal node
//     Eigen::Vector2i startIndex;
//     Eigen::Vector2i goalIndex;
//     CoordinateConverter::getInstance().cvtSceneToGridmap(agent.thisObject->center, startIndex);
//     CoordinateConverter::getInstance().cvtSceneToGridmap(task.goalPosition, goalIndex);
//     GridNode* startNode = &gridMap[startIndex];
//     startNode->position = agent.thisObject->center;
//     startNode->yaw = agent.thisObject->rotation;
//     GridNode goalNode;
//     goalNode.position = task.goalPosition;
//     goalNode.yaw = task.goalRotation;
//     goalNode.index = goalIndex;

//     /// create a priority queue
//     std::multimap<double, GridNode*> openSet;

//     /// A* algorithm support functions
//     /// TODO: implement the A* algorithm
//     auto heuristic = [&goalNode](GridNode* node) -> double
//     {
//         return (node->position - goalNode.position).norm();
//     };
//     struct NodeInfo
//     {
//         double cost = 0;
//         double yaw = 0;
//     };
//     auto getNeighbours = [&gridMap, &mainObjects](GridNode* current, std::vector<GridNode*>& neighbours, std::vector<NodeInfo>& neighboursInfo)
//     {
//         neighbours.clear();
//         neighboursInfo.clear();

//         const std::vector<double> yawList = {0, M_PI_4, M_PI_2, 3 * M_PI_4, M_PI, -3 * M_PI_4, -M_PI_2, -M_PI_4};
//         const std::vector<Eigen::Vector2i> indexList = {
//             {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
//         };

//         auto checkCollision = [&gridMap, &mainObjects](GridNode* node) -> bool
//         {
//             for (auto obj : mainObjects)
//             {
//                 obj.center = node->position;
//                 obj.rotation = node->yaw;
//                 obj.needTransform = true;
//                 obj.updateGeometry();
//                 for (int i = 0; i < obj.verticesTransformed.size(); i++)
//                 {
//                     Eigen::Vector2d point = obj.verticesTransformed[i];
//                     Eigen::Vector2i pointGridmap;
//                     CoordinateConverter::getInstance().cvtSceneToGridmap(point, pointGridmap);
//                     // std::cout << point.x() << " " << point.y() << " | " << pointGridmap.x() << " " << pointGridmap.y() << std::endl;
//                     if (pointGridmap.x() >= 0 && pointGridmap.x() < gridMap.width && pointGridmap.y() >= 0 && pointGridmap.y() < gridMap.height)
//                     {
//                         if (gridMap[pointGridmap].isOccupied)
//                         {
//                             return true;
//                         }
//                     }
//                     else
//                     {
//                         return true;
//                     }
//                 }
//             }
//             return false;
//         };

//         /// get possible neighbours
//         std::vector<int> checkList;
//         for (int i = 0; i < 8; i++)
//         {
//             if (fabs(yawList[i] - current->yaw) < 1e-6)
//             {
//                 checkList.push_back(i);
//                 checkList.push_back((i + 8 + 1) % 8);
//                 checkList.push_back((i + 8 - 1) % 8);
//                 checkList.push_back((i + 4) % 8);
//                 checkList.push_back((i + 4 + 1) % 8);
//                 checkList.push_back((i + 4 - 1) % 8);
//                 break;
//             }
//         }

//         /// except: 
//         /// 1. the node is out of the map
//         /// 2. the node causes collision
//         /// 3. the node is already in the close set
//         for (auto &i : checkList)
//         {
//             Eigen::Vector2i neighbourIndex = current->index + indexList[i];
//             /// ecxcept 1
//             if (neighbourIndex.x() < 0 || neighbourIndex.x() >= gridMap.width || neighbourIndex.y() < 0 || neighbourIndex.y() >= gridMap.height)
//             {
//                 continue;
//             }
//             GridNode* neighbour = &gridMap[neighbourIndex];
//             /// except 3
//             if (neighbour->status == SearchStatus::STATUS_CLOSE)
//             {
//                 continue;
//             }
//             /// except 2
//             if (checkCollision(neighbour))
//             {
//                 continue;
//             }
//             neighbours.push_back(neighbour);
//             NodeInfo neighbourInfo;
//             neighbourInfo.cost = (neighbour->position - current->position).norm();
//             neighbourInfo.yaw = yawList[i];
//             neighboursInfo.push_back(neighbourInfo);
//         }

//     };

//     auto timeStart = std::chrono::high_resolution_clock::now();
//     /// A* algorithm
//     startNode->gCost = 0;
//     startNode->hCost = heuristic(startNode);
//     startNode->fCost = startNode->gCost + startNode->hCost;
//     startNode->status = SearchStatus::STATUS_OPEN;
//     startNode->iterator = openSet.insert(std::make_pair(startNode->fCost, startNode));

//     GridNode* currentNode = nullptr;
//     std::vector<GridNode*> neighbours;
//     std::vector<NodeInfo> neighboursInfo;

//     while (!openSet.empty())
//     {
//         result.iteration++;
//         currentNode = openSet.begin()->second;
//         openSet.erase(openSet.begin());
//         currentNode->status = SearchStatus::STATUS_CLOSE;

//         if (currentNode->equal(goalNode, gridResolution))
//         {
//             result.success = true;
//             result.cost = currentNode->gCost;
//             /// TODO: reconstruct the path
//             GridNode* node = currentNode;
//             AgentAction goalAction(AgentAction::ACTION_SIMPLE_TRANSFER, agent.name, goalNode.position, goalNode.yaw);
//             result.actions.push_back(goalAction);
//             while (node != nullptr)
//             {
//                 AgentAction action(AgentAction::ACTION_SIMPLE_TRANSFER, agent.name, node->position, node->yaw);
//                 result.actions.push_back(action);
//                 node = node->parent;
//             }
//             std::reverse(result.actions.begin(), result.actions.end());
//             break;
//         }

//         getNeighbours(currentNode, neighbours, neighboursInfo);
//         for (int i = 0; i < neighbours.size(); i++)
//         {
//             GridNode* neighbour = neighbours[i];
//             NodeInfo neighbourInfo = neighboursInfo[i];
//             double newgCost = currentNode->gCost + neighbourInfo.cost;
//             if (neighbour->status == SearchStatus::STATUS_UNPROCESSED)
//             {
//                 neighbour->gCost = newgCost;
//                 neighbour->hCost = heuristic(neighbour);
//                 neighbour->fCost = neighbour->gCost + neighbour->hCost;
//                 neighbour->parent = currentNode;
//                 neighbour->status = SearchStatus::STATUS_OPEN;
//                 neighbour->yaw = neighbourInfo.yaw;
//                 neighbour->iterator = openSet.insert(std::make_pair(neighbour->fCost, neighbour));
//             }
//             else if (neighbour->status == SearchStatus::STATUS_OPEN &&
//                      newgCost < neighbour->gCost)
//             {
//                 openSet.erase(neighbour->iterator);
//                 neighbour->gCost = newgCost;
//                 neighbour->fCost = neighbour->gCost + neighbour->hCost;
//                 neighbour->parent = currentNode;
//                 neighbour->yaw = neighbourInfo.yaw;
//                 neighbour->iterator = openSet.insert(std::make_pair(neighbour->fCost, neighbour));
//             }
//         }
//     }

//     auto timeEnd = std::chrono::high_resolution_clock::now();
//     result.time = std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeStart).count() * 1e-6;
//     return result;
// }

void Planner::configure(Agent agent)
{
    this->agent = agent;
    /// geometry data
    double minX = -agent.scene->width / 2;
    double minY = -agent.scene->height / 2;
    double maxX = agent.scene->width / 2;
    double maxY = agent.scene->height / 2;
    int gridWidth = agent.scene->width / agent.scene->resolution;
    int gridHeight = agent.scene->height / agent.scene->resolution;
    double gridResolution = agent.scene->resolution;
    /// set objects
    this->staticObjects.clear();
    this->attachedObjects.clear();
    this->relativeTransforms.clear();
    this->agentObject = agent.thisObject;
    this->attachedObjects = agent.attachedObjects;
    this->relativeTransforms = agent.relativeTransforms;

    for (auto &object: agent.scene->sceneObjects)
    {
        if (object.name != agent.name && 
            std::find_if(agent.attachedObjects.begin(), agent.attachedObjects.end(), 
                [&object](SceneObject attachedObject) { return object.name == attachedObject.name; }) == agent.attachedObjects.end())
        {
            this->staticObjects.push_back(object);
        }
    }

    /// create a grid map
    this->gridMap.init(gridWidth, gridHeight, gridResolution);
    for (int i = 0; i < staticObjects.size(); i++)
    {
        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double maxX = -std::numeric_limits<double>::max();
        double maxY = -std::numeric_limits<double>::max();
        for (int j = 0; j < staticObjects.at(i).verticesTransformed.size(); j++)
        {
            double x = staticObjects.at(i).verticesTransformed[j].x();
            double y = staticObjects.at(i).verticesTransformed[j].y();
            minX = std::min(minX, x);
            minY = std::min(minY, y);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
        }
        for (double x = minX; x <= maxX; x += gridResolution)
        {
            for (double y = minY; y <= maxY; y += gridResolution)
            {
                Eigen::Vector2d point(x, y);
                Eigen::Vector2i pointGridmap;
                CoordinateConverter::getInstance().cvtSceneToGridmap(point, pointGridmap);
                if (staticObjects.at(i).containPoint(point) && 
                    pointGridmap.x() >= 0 && pointGridmap.x() < gridWidth && 
                    pointGridmap.y() >= 0 && pointGridmap.y() < gridHeight)
                {
                    gridMap[pointGridmap].isOccupied = true;
                    gridMap[pointGridmap].whoOccupied = i;
                }
            }
        }
    }
}

PlanResult Planner::plan(SimpleTransferTask task)
{
    auto timeStart = std::chrono::high_resolution_clock::now();
    PlanResult result;
    reset();

    /// create a start node and a goal node
    Eigen::Vector2i startIndex;
    Eigen::Vector2i goalIndex;
    CoordinateConverter::getInstance().cvtSceneToGridmap(agent.thisObject.center, startIndex);
    CoordinateConverter::getInstance().cvtSceneToGridmap(task.goalPosition, goalIndex);
    GridNode* startNode = &gridMap[startIndex];
    startNode->position = agent.thisObject.center;
    startNode->yaw = agent.thisObject.rotation;
    GridNode goalNode;
    goalNode.position = task.goalPosition;
    goalNode.yaw = task.goalRotation;
    goalNode.index = goalIndex;

    /// A* algorithm
    startNode->gCost = 0;
    startNode->hCost = heuristic(startNode, &goalNode);
    startNode->fCost = startNode->gCost + startNode->hCost;
    startNode->status = SearchStatus::STATUS_OPEN;
    startNode->iterator = openSet.insert(std::make_pair(startNode->fCost, startNode));

    GridNode* currentNode = nullptr;
    std::vector<GridNode*> neighbours;
    std::vector<NodeInfo> neighboursInfo;

    while (!openSet.empty())
    {
        result.iteration++;
        currentNode = openSet.begin()->second;
        openSet.erase(openSet.begin());
        currentNode->status = SearchStatus::STATUS_CLOSE;

        if (currentNode->equal(goalNode, gridMap.resolution))
        {
            result.success = true;
            result.cost = currentNode->gCost;

            GridNode* node = currentNode;
            AgentAction goalAction(AgentAction::ACTION_SIMPLE_TRANSFER, agent.name, goalNode.position, goalNode.yaw);
            result.actions.push_back(goalAction);
            while (node != nullptr)
            {
                AgentAction action(AgentAction::ACTION_SIMPLE_TRANSFER, agent.name, node->position, node->yaw);
                result.actions.push_back(action);
                node = node->parent;
            }
            std::reverse(result.actions.begin(), result.actions.end());
            break;
        }

        getNeighbours(currentNode, neighbours, neighboursInfo);
        for (int i = 0; i < neighbours.size(); i++)
        {
            GridNode* neighbour = neighbours[i];
            NodeInfo neighbourInfo = neighboursInfo[i];
            double newgCost = currentNode->gCost + neighbourInfo.cost;
            if (neighbour->status == SearchStatus::STATUS_UNPROCESSED)
            {
                neighbour->gCost = newgCost;
                neighbour->hCost = heuristic(neighbour, &goalNode);
                neighbour->fCost = neighbour->gCost + neighbour->hCost;
                neighbour->parent = currentNode;
                neighbour->status = SearchStatus::STATUS_OPEN;
                neighbour->yaw = neighbourInfo.yaw;
                neighbour->iterator = openSet.insert(std::make_pair(neighbour->fCost, neighbour));
            }
            else if (neighbour->status == SearchStatus::STATUS_OPEN &&
                     newgCost < neighbour->gCost)
            {
                openSet.erase(neighbour->iterator);
                neighbour->gCost = newgCost;
                neighbour->fCost = neighbour->gCost + neighbour->hCost;
                neighbour->parent = currentNode;
                neighbour->yaw = neighbourInfo.yaw;
                neighbour->iterator = openSet.insert(std::make_pair(neighbour->fCost, neighbour));
            }
        }
    }

    auto timeEnd = std::chrono::high_resolution_clock::now();
    result.time = std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeStart).count() * 1e-6;
    return result;
}

PlanResult Planner::plan(MoveObjectTask task)
{
    auto timeStart = std::chrono::high_resolution_clock::now();
    PlanResult result;
    reset();

    auto timeEnd = std::chrono::high_resolution_clock::now();
    result.time = std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeStart).count() * 1e-6;
    return result;
}

void Planner::reset()
{
    gridMap.releaseChangedNodes();
    openSet.clear();
}

double Planner::heuristic(GridNode *node, GridNode *goal)
{
    return (node->position - goal->position).norm();
}

bool Planner::checkCollision(GridNode *node)
{
    agentObject.center = node->position;
    agentObject.rotation = node->yaw;
    agentObject.needTransform = true;
    agentObject.updateGeometry();
    if (__check(agentObject)) return true;

    // for (auto & obj : attachedObjects)
    // {
    //     /// TODO: implement the collision check
    // }

    return false;
}

bool Planner::__check(SceneObject& object)
{
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    for (int i = 0; i < object.verticesTransformed.size(); i++)
    {
        double x = object.verticesTransformed[i].x();
        double y = object.verticesTransformed[i].y();
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
        minX = std::min(minX, x);
        minY = std::min(minY, y);
    }
    for (double x = minX; x <= maxX; x += gridMap.resolution)
    {
        for (double y = minY; y <= maxY; y += gridMap.resolution)
        {
            Eigen::Vector2d point(x, y);
            Eigen::Vector2i pointGridmap;
            CoordinateConverter::getInstance().cvtSceneToGridmap(point, pointGridmap);
            if (pointGridmap.x() < 0 || pointGridmap.x() >= gridMap.width ||
                pointGridmap.y() < 0 || pointGridmap.y() >= gridMap.height)
            {
                return true;
            }
            if (object.containPoint(point) && gridMap[pointGridmap].isOccupied)
            {
                return true;
            }
        }
    }
    // for (int i = 0; i < object.verticesTransformed.size(); i++)
    // {
    //     Eigen::Vector2d point = object.verticesTransformed[i];
    //     Eigen::Vector2i pointGridmap;
    //     CoordinateConverter::getInstance().cvtSceneToGridmap(point, pointGridmap);
    //     if (pointGridmap.x() >= 0 && pointGridmap.x() < gridMap.width && pointGridmap.y() >= 0 && pointGridmap.y() < gridMap.height)
    //     {
    //         if (gridMap[pointGridmap].isOccupied)
    //         {
    //             return true;
    //         }
    //     }
    //     else
    //     {
    //         return true;
    //     }
    // }
    return false;
}

void Planner::getNeighbours(GridNode *current, std::vector<GridNode *> &neighbours, std::vector<NodeInfo> &neighboursInfo)
{
    neighbours.clear();
    neighboursInfo.clear();

    const std::vector<double> yawList = {0, M_PI_4, M_PI_2, 3 * M_PI_4, M_PI, -3 * M_PI_4, -M_PI_2, -M_PI_4};
    const std::vector<Eigen::Vector2i> indexList = {
        {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}
    };

    /// get possible neighbours
    std::vector<int> checkList;
    for (int i = 0; i < 8; i++)
    {
        if (fabs(yawList[i] - current->yaw) < 1e-6)
        {
            checkList.push_back(i);
            checkList.push_back((i + 8 + 1) % 8);
            checkList.push_back((i + 8 - 1) % 8);
            checkList.push_back((i + 4) % 8);
            checkList.push_back((i + 4 + 1) % 8);
            checkList.push_back((i + 4 - 1) % 8);
            break;
        }
    }

    /// except: 
    /// 1. the node is out of the map
    /// 2. the node causes collision
    /// 3. the node is already in the close set
    for (auto &i : checkList)
    {
        Eigen::Vector2i neighbourIndex = current->index + indexList[i];
        /// ecxcept 1
        if (neighbourIndex.x() < 0 || neighbourIndex.x() >= gridMap.width || neighbourIndex.y() < 0 || neighbourIndex.y() >= gridMap.height)
        {
            continue;
        }
        GridNode* neighbour = &gridMap[neighbourIndex];
        /// except 3
        if (neighbour->status == SearchStatus::STATUS_CLOSE)
        {
            continue;
        }
        /// except 2
        if (checkCollision(neighbour))
        {
            continue;
        }
        neighbours.push_back(neighbour);
        NodeInfo neighbourInfo;
        neighbourInfo.cost = (neighbour->position - current->position).norm();
        neighbourInfo.yaw = yawList[i];
        neighboursInfo.push_back(neighbourInfo);
    }
}
