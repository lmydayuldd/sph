#ifndef SPRING_H
#define SPRING_H

//class Particle;
#include "machine/particle.h"

class Spring {
public:
    //static vector<Spring> springs = vector<Spring>();
    //Particle p1;
    Particle* p2;
    float ks, d, kd;

    ~Spring();
    Spring(/*Particle p1, */Particle* p2, float ks, float d, float kd);
};

#endif // SPRING_H
