#ifndef AGENT_REPRESENT_HPP
#define AGENT_REPRESENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <map>

#include "AgentTask.hpp"
#include "AgentAction.hpp"
#include "SceneRepresent.hpp"

class Agent
{
public: 
    Agent(std::string name, std::string sceneFile);

public:
    std::string name;
    SceneObject *thisObject;
    std::vector<SceneObject *> attachedObjects;
    std::vector<Eigen::Matrix3d> relativeTransforms;
    Scene scene;
    //PathPlan planner;
};

#endif // AGENT_REPRESENT_HPP