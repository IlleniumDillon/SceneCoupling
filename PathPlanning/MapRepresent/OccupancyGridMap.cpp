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
