#include <iostream>
#include "testmain.hpp"

int main() 
{
    Scene scene(SCENE_FILE_PATH);
    OccupancyGridMap map;
    scene.generateMap(map, 0.02);
    map.dilate();
    // VoronoiMap voronoiMap(map);
    ConnectGraph connectGraph(map);
    visualizeScene(scene, 0.02, "Scene");
    return 0;
}