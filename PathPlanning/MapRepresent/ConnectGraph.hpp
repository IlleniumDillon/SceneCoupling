#ifndef CONNECTGRAPH_HPP
#define CONNECTGRAPH_HPP

#include "OccupancyGridMap.hpp"

typedef struct
{
    
}connectState;

class ConnectGraph
{
public:
    ConnectGraph();
    ConnectGraph(const ConnectGraph &other);
    ConnectGraph(OccupancyGridMap &map);
    ConnectGraph &operator=(const ConnectGraph &other);
    ~ConnectGraph();
public:
    /// @brief number of connect area of the grid
    /// -2: in open list
    /// -1: unknown
    /// 0: obstacle
    /// n: connected area n
    int8_t **connectGridMap;
    int indexWidth; //the number of cells in the width of the map   (cols, horizontal)
    int indexHeight; //the number of cells in the height of the map (rows, vertical)
    int connectAreaNum; //the number of connected area
private:    
    int generateConnectGridMap(OccupancyGridMap &map);
    int generayeConnectGraph();
};

#endif // CONNECTGRAPH_HPP