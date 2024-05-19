#ifndef VISUALIZE_HPP
#define VISUALIZE_HPP

#include <MapRepresent.hpp>
#include <SceneRepresent.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>


void visualizeScene(Scene &scene, double res = 0.05, const std::string &windowName = "Scene");
void visualizeMap(OccupancyGridMap &map, const std::string &windowName = "Map");
void visualizeMap(VoronoiMap &map, const std::string &windowName = "Map");
void visualizePath(Path &path, const std::string &windowName = "Path");

#endif // VISUALIZE_HPP