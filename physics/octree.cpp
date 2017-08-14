#include "octree.h"

#include "machine/particle.h"
#include "physics/vector.h"
#include "util/settings.h"

double Octree::arena_diameter = Settings::ARENA_DIAMETER;
double Octree::cell_diameter = Settings::MESH_CELL_DIAMETER;
unsigned Octree::cell_count = arena_diameter / cell_diameter;
unsigned Octree::max_depth = floor(log(cell_count));
Octree *Octree::root = nullptr;

Octree::Octree()
{
    root = this;
    center[0] = 0.;
    center[1] = 0.;
    center[2] = 0.;
    Octree(0);
}

Octree::Octree(unsigned depth)
{
    for (unsigned i = 0; i < 8; ++i)
    {
        depth += 1;
        if (depth < max_depth) {
            *corners[i] = Octree(depth);
            if   (i     < 4) (*corners[i]).center[0] = center[0] - Settings::ARENA_DIAMETER / (2 * pow(2, depth));
            else             (*corners[i]).center[0] = center[0] - Settings::ARENA_DIAMETER / (2 * pow(2, depth));
            if   (i % 4 < 2) (*corners[i]).center[1] = center[1] - Settings::ARENA_DIAMETER / (2 * pow(2, depth));
            else             (*corners[i]).center[1] = center[1] - Settings::ARENA_DIAMETER / (2 * pow(2, depth));
            if   (i % 2 < 1) (*corners[i]).center[2] = center[2] - Settings::ARENA_DIAMETER / (2 * pow(2, depth));
            else             (*corners[i]).center[2] = center[2] - Settings::ARENA_DIAMETER / (2 * pow(2, depth));
        }
    }
}

void Octree::empty()
{
    particles.clear();
    for (unsigned i = 0; i < 8; ++i)
    {
        corners[i]->empty();
    }
}

void Octree::distributeParticles()
{
    root->empty();

    for (unsigned i = 0; i < Particle::flows.size(); ++i)
    {
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
        {
            root->fitParticle(Particle::flows[i][j]);
        }
    }
}

void Octree::fitParticle(Particle* p)
{
    unsigned corner = 0;
    if (p->r->x <= center[0]) corner  = 0; // L
    else                      corner  = 4; // R
    if (p->r->y <= center[1]) corner += 0; // D
    else                      corner += 2; // U
    if (p->r->z <= center[2]) corner += 0; // B
    else                      corner += 1; // F

    if (current_depth == max_depth - 1)
    {
        p->cube = this;
        corners[corner]->particles.push_back(p);
    }
    else
    {
        corners[corner]->fitParticle(p);
    }
}

vector<Particle*> Octree::getNeighbours(Particle* p)
{
    vector<Particle*> neighbours;
    for (unsigned i = 0; i < particles.size(); ++i)
    {
        if (particles[i] != p) {
            neighbours.push_back(particles[i]);
        }
    }
    return neighbours;
}
