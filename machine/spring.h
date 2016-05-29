#ifndef SPRING_H
#define SPRING_H

//class Particle;
#include "machine/particle.h"

class Spring {
public:
    Particle* p2;
    float ks, d, kd;

    ~Spring();
    Spring(
        Particle* p2,
        float ks,
        float d,
        float kd
    );
};

#endif // SPRING_H
