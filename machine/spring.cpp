#include "machine/spring.h"

Spring::~Spring()
{
    delete p2;
}

/**
 * Basing on Hooke's Force.
 * @param p2 Second particle belonging to the spring.
 * @param ks Spring force.
 * @param d  Distance, spring length.
 * @param kd Spring force damping.
 */
Spring::Spring(Particle* p2, float ks, float d, float kd) {
    this->p2 = p2;
    this->ks = ks;
    this->d  = d;
    this->kd = kd;
}
