#ifndef AGENT_TASK_HPP
#define AGENT_TASK_HPP

#include <iostream>
#include <vector>
#include <string>

#include "eigen3/Eigen/Eigen"

class AgentTask
{
public:
    enum TaskType
    {
        TASK_SIMPLE_TRANSFER,
        TASK_MOVE_OBJECT,
    };
};

class SimpleTransferTask : public AgentTask
{
public:
    SimpleTransferTask() {}
    SimpleTransferTask(std::string agentName, Eigen::Vector2d goalPosition, double goalRotation)
        : agentName(agentName), goalPosition(goalPosition), goalRotation(goalRotation) {}
    SimpleTransferTask(std::string filePath)
    {
        // Read task from file
    }
public:
    std::string agentName;
    Eigen::Vector2d goalPosition;
    double goalRotation;
};

class MoveObjectTask : public AgentTask
{
public:
    MoveObjectTask() {}
    MoveObjectTask(std::string objectName, Eigen::Vector2d goalPosition, double goalRotation)
        : objectName(objectName), goalPosition(goalPosition), goalRotation(goalRotation) {}
    MoveObjectTask(std::string filePath)
    {
        // Read task from file
    }
public:
    std::string objectName;
    Eigen::Vector2d goalPosition;
    double goalRotation;
};

#endif // AGENT_TASK_HPP