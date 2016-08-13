#ifndef COMPUTER_H
#define COMPUTER_H

#include "machine/particle.h"

class Computer
{
public:
    static Computer* currentComputer;

    Computer();

    static void evaluateForces(const Particle& p);
    static void evaluateForces();
    static void evaluateSPHForces();
    static void computeVectors(const Particle& p, float dt);
    static void computeVectors(float dt);
    static void Euler(const Particle& p, float dt);
    static void Euler(float dt);
    static void MidPoint(const Particle& p, float dt);
    static void MidPoint(float dt);
    static void RangeKutta();
    static void RKV4();

    void loop();
    void flagUp(int i, int j, int flag);
    bool flagIs(int i, int j, int flag);
};

#endif // COMPUTER_H
