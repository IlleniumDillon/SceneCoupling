#ifndef OCCUPANCYGRIDMAP_HPP
#define OCCUPANCYGRIDMAP_HPP

#include "MapObject.hpp"

class OccupancyGridMap  : public MapObject
{
public:
    OccupancyGridMap();
    OccupancyGridMap(const OccupancyGridMap& other);
    OccupancyGridMap& operator=(const OccupancyGridMap& other);
    uint8_t& operator()(int x, int y);
    uint8_t operator()(int x, int y) const;
    ~OccupancyGridMap();

    void setMapSize(double width, double height, double resolution);
    void setOrigin(double x, double y);

public:
    Eigen::Vector2d origin; //the coordinates of the top-left corner of the map
    double resolution; //the size of each cell in the map
    double mapWidth; //the width of the map (in meters, x-axis, cols, horizontal)
    double mapHeight; //the height of the map   (in meters, y-axis, rows, vertical)
    int indexWidth; //the number of cells in the width of the map   (cols, horizontal)
    int indexHeight; //the number of cells in the height of the map (rows, vertical)

    uint8_t** map; //the map itself, a 2D array of uint8_t values
};

#endif // OCCUPANCYGRIDMAP_HPP