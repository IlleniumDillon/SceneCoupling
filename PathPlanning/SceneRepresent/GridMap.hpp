#ifndef GRIDMAP_HPP
#define GRIDMAP_HPP

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include "eigen3/Eigen/Eigen"

class GridMap
{
public:
    GridMap(){};
    GridMap(int width, int height, double resolution);
    ~GridMap();
    GridMap& operator=(const GridMap &gridMap);
    uint8_t& operator()(int x, int y);

    void dilate(double radius);
public:
    int width = 0;
    int height = 0;
    double resolution = 0;
    uint8_t **gridMap = nullptr;
};


#endif // GRIDMAP_HPP