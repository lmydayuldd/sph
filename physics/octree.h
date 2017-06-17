#ifndef OCTREE_H
#define OCTREE_H

class Particle;

#include <vector>
#include <cmath>

using namespace std;

class Octree
{
public:
    static Octree *root;
    static double arena_diameter;
    static double cell_diameter;
    static unsigned int cell_count;
    static unsigned int max_depth;
    unsigned int current_depth;

    Octree();

    static void distributeParticles();
    void fitParticle(Particle* p);
    vector<Particle*> getNeighbours(Particle* p);

private:
    Octree *corners[8];
    vector<Particle*> particles;
    double center[3];

    Octree(unsigned int depth);

    void empty();
};

#endif // OCTREE_H
