#include "OccupancyGridMap.hpp"

OccupancyGridMap::OccupancyGridMap()
{
    this->origin = Eigen::Vector2d(0, 0);
    this->indexHeight = 0;
    this->indexWidth = 0;
    this->mapHeight = 0;
    this->mapWidth = 0;
    this->resolution = 0;
    this->map = nullptr;
}

OccupancyGridMap::OccupancyGridMap(const OccupancyGridMap &other)
{
    this->origin = other.origin;
    this->indexHeight = other.indexHeight;
    this->indexWidth = other.indexWidth;
    this->mapHeight = other.mapHeight;
    this->mapWidth = other.mapWidth;
    this->resolution = other.resolution;
    this->map = new uint8_t *[this->indexHeight];
    for (int i = 0; i < this->indexHeight; i++)
    {
        this->map[i] = new uint8_t[this->indexWidth];
        for (int j = 0; j < this->indexWidth; j++)
        {
            this->map[i][j] = other.map[i][j];
        }
    }
}

OccupancyGridMap &OccupancyGridMap::operator=(const OccupancyGridMap &other)
{
    if (this == &other)
    {
        return *this;
    }
    if (this->map != nullptr)
    {
        for (int i = 0; i < this->indexHeight; i++)
        {
            delete[] this->map[i];
        }
        delete[] this->map;
    }
    this->origin = other.origin;
    this->indexHeight = other.indexHeight;
    this->indexWidth = other.indexWidth;
    this->mapHeight = other.mapHeight;
    this->mapWidth = other.mapWidth;
    this->resolution = other.resolution;
    this->map = new uint8_t *[this->indexHeight];
    for (int i = 0; i < this->indexHeight; i++)
    {
        this->map[i] = new uint8_t[this->indexWidth];
        for (int j = 0; j < this->indexWidth; j++)
        {
            this->map[i][j] = other.map[i][j];
        }
    }
    return *this;
}

uint8_t &OccupancyGridMap::operator()(int x, int y)
{
    if (x < 0 || x >= this->indexWidth || y < 0 || y >= this->indexHeight)
    {
        throw std::out_of_range("in function 'uint8_t &OccupancyGridMap::operator()(int x, int y)', Out of range");
    }
    return this->map[y][x];
}

uint8_t OccupancyGridMap::operator()(int x, int y) const
{
    if (x < 0 || x >= this->indexWidth || y < 0 || y >= this->indexHeight)
    {
        return 1;
    }
    return this->map[y][x];
}

OccupancyGridMap::~OccupancyGridMap()
{
    if (this->map != nullptr)
    {
        for (int i = 0; i < this->indexHeight; i++)
        {
            delete[] this->map[i];
        }
        delete[] this->map;
    }
}

void OccupancyGridMap::setMapSize(double width, double height, double resolution)
{
    this->mapWidth = width;
    this->mapHeight = height;
    this->resolution = resolution;
    this->indexWidth = width / resolution;
    this->indexHeight = height / resolution;
    if (this->map != nullptr)
    {
        for (int i = 0; i < this->indexHeight; i++)
        {
            delete[] this->map[i];
        }
        delete[] this->map;
    }
    this->map = new uint8_t *[this->indexHeight];
    for (int i = 0; i < this->indexHeight; i++)
    {
        this->map[i] = new uint8_t[this->indexWidth];
        for (int j = 0; j < this->indexWidth; j++)
        {
            this->map[i][j] = 0;
        }
    }
}

void OccupancyGridMap::setOrigin(double x, double y)
{
    this->origin = Eigen::Vector2d(x, y);
}


bool OccupancyGridMap::checkIntersect(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c, Eigen::Vector2d d)
{
    if(
        std::max(c.x(), d.x()) < std::min(a.x(), b.x()) || 
        std::max(a.x(), b.x()) < std::min(c.x(), d.x()) || 
        std::max(c.y(), d.y()) < std::min(a.y(), b.y()) || 
        std::max(a.y(), b.y()) < std::min(c.y(), d.y())) 
    {
        return false;
    }

    auto cross = [](Eigen::Vector2d a, Eigen::Vector2d b) -> double
    {
        return a.x() * b.y() - a.y() * b.x();
    };

    if (cross(a - d, c - d) * cross(b - d, c - d) > 0 ||
        cross(d - b, a - b) * cross(c - b, a - b) > 0) 
    {
        return  false;
    }


    return true;
}

void OccupancyGridMap::polygon(const std::vector<Eigen::Vector2d> &vertices)
{
    std::vector<Eigen::Vector2i> polygonVertices;
    for (auto &vertex : vertices)
    {
        polygonVertices.push_back(Eigen::Vector2i((vertex.x() - this->origin.x()) / this->resolution, (vertex.y() - this->origin.y()) / this->resolution));
    }

    int minX = INT_MAX, minY = INT_MAX, maxX = INT_MIN, maxY = INT_MIN;
    for (auto &vertex : polygonVertices)
    {
        minX = std::min(minX, vertex.x());
        minY = std::min(minY, vertex.y());
        maxX = std::max(maxX, vertex.x());
        maxY = std::max(maxY, vertex.y());
    }

    for (int i = minY; i <= maxY; i++)
    {
        for (int j = minX; j <= maxX; j++)
        {
            if (i < 0 || i >= this->indexHeight || j < 0 || j >= this->indexWidth)
            {
                continue;
            }

            Eigen::Vector2d point(j * this->resolution + this->origin.x(), i * this->resolution + this->origin.y());
            bool inside = false;
            for (int k = 0; k < polygonVertices.size(); k++)
            {
                Eigen::Vector2d a = vertices[k];
                Eigen::Vector2d b = vertices[(k + 1) % polygonVertices.size()];
                if (checkIntersect(a, b, point, Eigen::Vector2d(maxX, point.y())))
                {
                    inside = !inside;
                }
            }
            if (inside)
            {
                this->map[i][j] = 1;
            }
        }
    }
}
