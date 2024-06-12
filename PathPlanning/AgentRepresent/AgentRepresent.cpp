#include "AgentRepresent.hpp"

Agent::Agent(std::string name, std::string sceneFile)
{
    this->name = name;
    this->scene = Scene(sceneFile);
    this->thisObject = this->scene[name];
}
