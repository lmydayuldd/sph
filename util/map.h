#ifndef MAP_H
#define MAP_H

#include <vector>

class Map
{
private:
    static std::vector<std::vector<char>> map;

    static void reset();
    static void import(unsigned char mapSetup);

    static unsigned mapWidth;
    static unsigned mapHeight;
    static unsigned obstacleCount;
    static unsigned particleCount;
    static unsigned ghostCount;

public:
    static void generate();

    Map();
};

#endif // MAP_H
