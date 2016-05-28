#ifndef ROPE_H
#define ROPE_H

#include <vector>

class Vector;

#include "machine/particle.h"

class Rope
{
private:
    std::vector<Particle>* flow;

public:
    ~Rope();
    Rope(
        Vector* start,
        Vector* end,
        int knots,
        float ks,
        float d,
        float kd,
        int strength
    );

    void paint();
};

#endif // ROPE_H
