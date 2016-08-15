#ifndef GRID_H
#define GRID_H

class Particle;

#include <vector>

using namespace std;

class Grid
{
public:
    static vector<vector<vector<vector<Particle*>>>> grid;
    static double arena_diameter;
    static double cell_diameter;
    static unsigned int cell_count;

    static void init();

    Grid();

    static void update();
    static void fitParticle(Particle* p);
};

#endif // GRID_H
