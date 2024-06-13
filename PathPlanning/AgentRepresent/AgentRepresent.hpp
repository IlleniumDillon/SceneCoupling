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
    Agent(){}
    Agent(std::string name, Scene* scene);
    void attachObject(std::string objectName);
    void detachObject(std::string objectName);

public:
    std::string name;
    SceneObject thisObject;
    std::vector<SceneObject> attachedObjects;
    std::vector<Eigen::Matrix3d> relativeTransforms;
    Scene* scene;
    //PathPlan planner;
};

#endif // AGENT_REPRESENT_HPP