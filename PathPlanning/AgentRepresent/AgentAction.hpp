#ifndef AGENT_ACTION_HPP
#define AGENT_ACTION_HPP

#include <iostream>
#include <vector>
#include <string>

#include "eigen3/Eigen/Eigen"

class AgentAction
{
public:
    enum ActionType
    {
        ACTION_NONE,
        ACTION_SIMPLE_TRANSFER,
        ACTION_ATTACH,
        ACTION_DETACH,
    };

public:
    AgentAction()
        : type(ACTION_NONE), agentName(""), objectName("") {}
    AgentAction(ActionType type, std::string agentName, std::string objectName)
        : type(type), agentName(agentName), objectName(objectName) {}
    AgentAction(ActionType type, std::string agentName, Eigen::Vector2d position, double yaw)
        : type(type), agentName(agentName), position(position), yaw(yaw) {}

public:
    ActionType type;
    std::string agentName;
    /// data for transfer
    Eigen::Vector2d position;
    double yaw;
    /// data for attach and detach
    std::string objectName;
};


#endif // AGENT_ACTION_HPP