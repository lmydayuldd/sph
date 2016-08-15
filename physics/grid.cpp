#include "grid.h"

#include <omp.h>

#include "machine/particle.h"
#include "physics/vector.h"
#include "util/settings.h"

vector<vector<vector<vector<Particle*>>>> Grid::grid;
double Grid::arena_diameter = Settings::ARENA_DIAMETER;
double Grid::cell_diameter = Settings::SPH_MESH_CELL_DIAMETER;
unsigned int Grid::cell_count = arena_diameter / cell_diameter;

void Grid::init()
{
    grid = vector<vector<vector<vector<Particle*>>>>(
               cell_count, vector<vector<vector<Particle*>>>(
                   cell_count, vector<vector<Particle*>>(
                       cell_count, vector<Particle*>())));
}

Grid::Grid()
{
    init();
}

void Grid::update()
{
#pragma omp parallel for
    for (unsigned int i = 0; i < grid.size(); ++i)
        for (unsigned int j = 0; j < grid[i].size(); ++j)
            for (unsigned int k = 0; k < grid[i][j].size(); ++k)
                grid[i][j][k].clear();

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
    {
#pragma omp parallel for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
        {
            fitParticle(Particle::flows[i][j]);
        }
    }
}

void Grid::fitParticle(Particle* p)
{
    unsigned int x = (unsigned int) ((p->r->x + 10.) / cell_diameter);
    unsigned int y = (unsigned int) ((p->r->y + 10.) / cell_diameter);
    unsigned int z = (unsigned int) ((p->r->z + 10.) / cell_diameter);
    p->cell[0] = x;
    p->cell[1] = y;
    p->cell[2] = z;
//#pragma omp single
    grid[x][y][z].push_back(p);
}
