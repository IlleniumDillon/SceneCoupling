#include <iostream>
#include "testmain.hpp"

int main() 
{
    Scene scene(SCENE_FILE_PATH);
    OccupancyGridMap map;
    scene.generateMap(map, 0.02);
    VoronoiMap voronoiMap(map);
    //visualizeMap(map, "Map");
    visualizeScene(scene, 0.02, "Scene");
    return 0;
}