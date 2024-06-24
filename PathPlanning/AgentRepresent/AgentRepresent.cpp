#include "AgentRepresent.hpp"

Agent::Agent(std::string name, Scene* scene)
{
    this->name = name;
    this->thisObject = *(*scene)[name];
    this->scene = scene;
}

void Agent::attachObject(std::string objectName, Eigen::Vector3d agentPose, Eigen::Vector3d objectPose)
{
    SceneObject object = *(*scene)[objectName];
    attachedObjects.push_back(object);
    Eigen::Matrix3d agentTF;
    Eigen::Matrix3d objectTF;
    agentTF << cos(agentPose[2]), -sin(agentPose[2]), agentPose[0],
               sin(agentPose[2]), cos(agentPose[2]), agentPose[1],
               0, 0, 1;
    objectTF << cos(objectPose[2]), -sin(objectPose[2]), objectPose[0],
                sin(objectPose[2]), cos(objectPose[2]), objectPose[1],
                0, 0, 1;
    Eigen::Matrix3d relativeTransform = objectTF * agentTF.inverse();
    relativeTransforms.push_back(relativeTransform);
    relativePositions.push_back({objectPose[0] - agentPose[0], objectPose[1] - agentPose[1]});
    relativeRotations.push_back(objectPose[2] - agentPose[2]);
}

void Agent::detachObject(std::string objectName)
{
}
