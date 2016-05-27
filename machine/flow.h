#ifndef FLOW_H
#define FLOW_H

#include <vector>

//class Particle;
#include "machine/particle.h"

class Flow
{
public:
    static std::vector<Flow> flows;

    std::vector<Particle> particles;

    Flow();
    Flow(int particlesCount);
};

#endif // FLOW_H
