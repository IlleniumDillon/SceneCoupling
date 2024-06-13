#ifndef VISUALIZE_HPP
#define VISUALIZE_HPP

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <chrono>

#include "eigen3/Eigen/Eigen"
#include "opencv2/opencv.hpp"

#include "SceneRepresent.hpp"
#include "AgentRepresent.hpp"
#include "PathPlan.hpp"

void visualize(Scene scene, double fps = 30);
void visualize(Scene scene, Agent agent, PlanResult result, double fps = 30);

#endif // VISUALIZE_HPP