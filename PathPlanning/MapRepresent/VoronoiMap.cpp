#include "VoronoiMap.hpp"

VoronoiMap::VoronoiMap()
{
    this->vertices.clear();
}

VoronoiMap::VoronoiMap(const VoronoiMap &other)
{
    this->vertices = other.vertices;
}

VoronoiMap::VoronoiMap(const OccupancyGridMap &map)
{
    /// step 1: delicat the map till no more dilation is needed

    /// step 2: mark all local maximums as vertices

    /// step 3: simplify the vertices and link them with edges
}

VoronoiMap &VoronoiMap::operator=(const VoronoiMap &other)
{
    if (this == &other)
    {
        return *this;
    }
    this->vertices = other.vertices;
    return *this;
}

VoronoiMap::~VoronoiMap()
{
    this->vertices.clear();
}
