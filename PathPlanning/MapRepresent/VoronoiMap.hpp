#ifndef VORONOIMAP_HPP
#define VORONOIMAP_HPP

#include "MapObject.hpp"

#include "OccupancyGridMap.hpp"
#include "SceneRepresent.hpp"

#define PREVIEW_FLAG 1
#define CENTER_DISTANCE 10

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

    class VoronoiGridNode
    {
    public:
        VoronoiGridNode()
        {
            position = Eigen::Vector2i(0, 0);
            flag = 0;
            numOfNeighbors = 0;
            comeForm = nullptr;
            isBackBone = false;
        }
        VoronoiGridNode(int x, int y)
        {
            position = Eigen::Vector2i(x, y);
            flag = 0;
            numOfNeighbors = 0;
            comeForm = nullptr;
            isBackBone = false;
        }
        Eigen::Vector2i position;
        int flag;
        int numOfNeighbors;
        VoronoiGridNode* comeForm;
        bool isBackBone;
    };

    int delicateAndCount(int** delicateMap, OccupancyGridMap& map);
    int getBackBone(int** delicateMap, OccupancyGridMap& map, std::vector<Eigen::Vector2i>& backBone);
    int generateVertices(std::vector<Eigen::Vector2i>& verticesList, OccupancyGridMap& map, std::vector<Eigen::Vector2i>& backBone);
    int generateGraph(std::vector<Eigen::Vector2i>& verticesList, OccupancyGridMap& map);

    std::vector<Eigen::Vector2i> drawLine(Eigen::Vector2i start, Eigen::Vector2i end);
    bool checkLineOccupancy(std::vector<Eigen::Vector2i> points, OccupancyGridMap& map);
};

#endif // VORONOIMAP_HPP