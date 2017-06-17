#include "grid.h"

#include <omp.h>

#include "machine/particle.h"
#include "physics/vector.h"
#include "util/operations.h"
#include "util/settings.h"

vector<vector<vector<vector<Particle*>>>> Grid::grid;
double Grid::arena_diameter = Settings::ARENA_DIAMETER;
double Grid::cell_diameter = Settings::SPH_MESH_CELL_DIAMETER;
unsigned int Grid::cell_count = arena_diameter / cell_diameter;

void Grid::init()
{
    grid = vector<vector<vector<vector<Particle*>>>>(
// TODO why  1?!!!!!!!!
               cell_count, vector<vector<vector<Particle*>>>(
                   cell_count, vector<vector<Particle*>>(
                       cell_count, vector<Particle*>())));
}

Grid::Grid()
{
    init();
}

void Grid::distributeParticles()
{
#pragma omp parallel for
    for (unsigned int i = 0; i < grid.size(); ++i)
        for (unsigned int j = 0; j < grid[i].size(); ++j)
            for (unsigned int k = 0; k < grid[i][j].size(); ++k)
                grid[i][j][k].clear();

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
    {
//#pragma omp parallel for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
        {
            fitParticle(Particle::flows[i][j]);
        }
    }
}

void Grid::fitParticle(Particle* p)
{
    double r = Settings::ARENA_DIAMETER / 2;
    //  TODO
    //   it shouldn't be done here
    //   on call of Grid::update() particles
    //   should already be inside the premises,
    //   in fact - they should always be
    // problem may be in: Walls:collide, >>Computer::collide<<, ...
    // Computer::Collide collides particles which may get out of bounds
    //   (and then get collided again and get former position out of bounds
    //    as well), before checking Wall collisions!!!
    p->r->limit(Settings::ARENA_DIAMETER);
//    if (fabs(p->r->x) > r
//     || fabs(p->r->y) > r
//     || fabs(p->r->z) > r) exit(13);
    unsigned int dx = (unsigned int) ((p->r->x + r) / cell_diameter);
    unsigned int dy = (unsigned int) ((p->r->y + r) / cell_diameter);
    unsigned int dz = (unsigned int) ((p->r->z + r) / cell_diameter);
    if (dx >= Grid::cell_count) dx = Grid::cell_count - 1;
    if (dy >= Grid::cell_count) dy = Grid::cell_count - 1;
    if (dz >= Grid::cell_count) dz = Grid::cell_count - 1;
    p->cell[0] = dx;
    p->cell[1] = dy;
    p->cell[2] = dz;
//#pragma omp single
    grid[dx][dy][dz].push_back(p);
}
