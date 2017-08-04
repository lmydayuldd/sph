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
    enum Setup : unsigned char {
        RANDOM_NON_MAP, DAM_BREAK_NON_MAP, DAM_BREAK_3D_NON_MAP,
        DAM_BREAK, DROPLET, VESSELS, DAM_FALL
    };

    static void generate();

    Map();
};

#endif // MAP_H
