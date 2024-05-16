#ifndef VORONOIMAP_HPP
#define VORONOIMAP_HPP

#include "MapObject.hpp"

#include "OccupancyGridMap.hpp"

class vertex;

typedef std::shared_ptr<vertex> vertexPtr;

class vertex
{
public:
    Eigen::Vector2d position;           // the position of the vertex
    std::vector<vertexPtr> neighbors;     // the neighbors of the vertex
    std::vector<Path> edges;        // the edges of the vertex to its neighbors
};

class VoronoiMap    : public MapObject
{
public:
    VoronoiMap();
    VoronoiMap(const VoronoiMap& other);
    VoronoiMap(const OccupancyGridMap& map);
    VoronoiMap& operator=(const VoronoiMap& other);
    ~VoronoiMap();
    
public:
    std::vector<vertexPtr> vertices; // the vertices of the Voronoi diagram

private:
};

#endif // VORONOIMAP_HPP