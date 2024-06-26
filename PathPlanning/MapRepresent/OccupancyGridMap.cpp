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

void OccupancyGridMap::dilate(int radius)
{
    if (radius <= 0)
    {
        return;
    }

    uint8_t **newMap = new uint8_t *[this->indexHeight];
    for (int i = 0; i < this->indexHeight; i++)
    {
        newMap[i] = new uint8_t[this->indexWidth];
        for (int j = 0; j < this->indexWidth; j++)
        {
            newMap[i][j] = this->map[i][j];
        }
    }

    for (int i = 0; i < this->indexHeight; i++)
    {
        for (int j = 0; j < this->indexWidth; j++)
        {
            if (this->map[i][j] == 1)
            {
                for (int k = -radius; k <= radius; k++)
                {
                    for (int l = -radius; l <= radius; l++)
                    {
                        if (i + k < 0 || i + k >= this->indexHeight || j + l < 0 || j + l >= this->indexWidth)
                        {
                            continue;
                        }
                        if (k * k + l * l <= radius * radius)
                        {
                            newMap[i + k][j + l] = 1;
                        }
                    }
                }
            }
        }
    }

    for (int i = 0; i < this->indexHeight; i++)
    {
        delete[] this->map[i];
    }
    delete[] this->map;
    this->map = newMap;
}

OccupancyGridMap OccupancyGridMap::dilate(int radius) const
{
    if (radius <= 0)
    {
        return *this;
    }

    OccupancyGridMap newMap;
    newMap.origin = this->origin;
    newMap.indexHeight = this->indexHeight;
    newMap.indexWidth = this->indexWidth;
    newMap.mapHeight = this->mapHeight;
    newMap.mapWidth = this->mapWidth;
    newMap.resolution = this->resolution;
    newMap.map = new uint8_t *[newMap.indexHeight];
    for (int i = 0; i < newMap.indexHeight; i++)
    {
        newMap.map[i] = new uint8_t[newMap.indexWidth];
        for (int j = 0; j < newMap.indexWidth; j++)
        {
            newMap.map[i][j] = this->map[i][j];
        }
    }

    for (int i = 0; i < newMap.indexHeight; i++)
    {
        for (int j = 0; j < newMap.indexWidth; j++)
        {
            if (this->map[i][j] == 1)
            {
                for (int k = -radius; k <= radius; k++)
                {
                    for (int l = -radius; l <= radius; l++)
                    {
                        if (i + k < 0 || i + k >= newMap.indexHeight || j + l < 0 || j + l >= newMap.indexWidth)
                        {
                            continue;
                        }
                        if (k * k + l * l <= radius * radius)
                        {
                            newMap.map[i + k][j + l] = 1;
                        }
                    }
                }
            }
        }
    }

    return newMap;
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

void OccupancyGridMap::polygon(const std::vector<Eigen::Vector2d> &vertices, uint8_t value)
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

    for (int i = minY - 1; i <= maxY + 1; i++)
    {
        for (int j = minX - 1; j <= maxX + 1; j++)
        {
            if (i < 0 || i >= this->indexHeight || j < 0 || j >= this->indexWidth)
            {
                continue;
            }

            Eigen::Vector2d point((j+0.14) * this->resolution + this->origin.x(), (i+0.14) * this->resolution + this->origin.y());
            int cross = 0;
            for (int k = 0; k < polygonVertices.size(); k++)
            {
                Eigen::Vector2d a = vertices[k];
                Eigen::Vector2d b = vertices[(k + 1) % polygonVertices.size()];
                if (checkIntersect(a, b, point, Eigen::Vector2d(maxX + 1, point.y())))
                {
                    cross++;
                }
            }
            if (cross % 2 == 1)
            {
                this->map[i][j] = value;
            }
        }
    }
}
