#include "machine/spring.h"

Spring::~Spring()
{
    delete p2;
}

Spring::Spring(Particle* p2, float ks, float d, float kd) {
    this->p2 = p2;
    this->ks = ks;
    this->d  = d;
    this->kd = kd;
}
