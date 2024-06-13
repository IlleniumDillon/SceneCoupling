#include "AgentRepresent.hpp"

Agent::Agent(std::string name, Scene* scene)
{
    this->name = name;
    this->thisObject = *(*scene)[name];
    this->scene = scene;
}

void Agent::attachObject(std::string objectName)
{
}

void Agent::detachObject(std::string objectName)
{
}
