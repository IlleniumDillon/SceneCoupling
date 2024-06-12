#include "GridMap.hpp"

GridMap::GridMap(int width, int height, double resolution)
{
    this->width = width;
    this->height = height;
    this->resolution = resolution;
    gridMap = new uint8_t *[height];
    for (int i = 0; i < height; i++)
    {
        gridMap[i] = new uint8_t[width];
        for (int j = 0; j < width; j++)
        {
            gridMap[i][j] = 0;
        }
    }
}

GridMap::~GridMap()
{
    if (gridMap != nullptr)
    {
        for (int i = 0; i < height; i++)
        {
            delete[] gridMap[i];
        }
        delete[] gridMap;
    }
}

GridMap &GridMap::operator=(const GridMap &gridMap)
{
    if (this == &gridMap)
    {
        return *this;
    }

    if (this->gridMap != nullptr)
    {
        for (int i = 0; i < height; i++)
        {
            delete[] this->gridMap[i];
        }
        delete[] this->gridMap;
    }

    this->width = gridMap.width;
    this->height = gridMap.height;
    this->resolution = gridMap.resolution;
    this->gridMap = new uint8_t *[height];
    for (int i = 0; i < height; i++)
    {
        this->gridMap[i] = new uint8_t[width];
        for (int j = 0; j < width; j++)
        {
            this->gridMap[i][j] = gridMap.gridMap[i][j];
        }
    }

    return *this;
}

uint8_t &GridMap::operator()(int x, int y)
{
    if (x < 0 || x >= width || y < 0 || y >= height)
    {
        /// exception: out of range
        std::cerr << "Out of range" << std::endl;
        throw std::out_of_range("Out of range");
    }
    return gridMap[y][x];
}

void GridMap::dilate(double radius)
{
    int dilateRadius = radius / resolution;
    uint8_t **gridMapCopy = new uint8_t *[height];
    for (int i = 0; i < height; i++)
    {
        gridMapCopy[i] = new uint8_t[width];
        for (int j = 0; j < width; j++)
        {
            gridMapCopy[i][j] = gridMap[i][j];
        }
    }

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (gridMapCopy[i][j] == 0)
            {
                for (int m = -dilateRadius; m <= dilateRadius; m++)
                {
                    for (int n = -dilateRadius; n <= dilateRadius; n++)
                    {
                        if (m == 0 && n == 0)
                        {
                            continue;
                        }
                        if (i + m >= 0 && i + m < height && j + n >= 0 && j + n < width)
                        {
                            if (gridMapCopy[i + m][j + n] == 1)
                            {
                                gridMap[i][j] = 1;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    for (int i = 0; i < height; i++)
    {
        delete[] gridMapCopy[i];
    }
    delete[] gridMapCopy;
}
