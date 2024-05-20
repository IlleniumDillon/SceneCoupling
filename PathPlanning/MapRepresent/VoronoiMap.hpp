#ifndef VORONOIMAP_HPP
#define VORONOIMAP_HPP

#include "MapObject.hpp"

#include "OccupancyGridMap.hpp"
#include "SceneRepresent.hpp"

#define PREVIEW_FLAG 0

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
    //VoronoiMap(const Scene& scene);
    VoronoiMap(OccupancyGridMap& map);
    VoronoiMap& operator=(const VoronoiMap& other);
    ~VoronoiMap();
    
public:
    std::vector<vertexPtr> vertices; // the vertices of the Voronoi diagram

private:
    int delicateAndCount(int** delicateMap, OccupancyGridMap& map);
    int getBackBone(int** delicateMap, OccupancyGridMap& map, std::vector<Eigen::Vector2i>& backBone);
    int generateVertices(int** delicateMap, OccupancyGridMap& map, std::vector<Eigen::Vector2i>& backBone);
};

#endif // VORONOIMAP_HPP