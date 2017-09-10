#ifndef COMPUTER_H
#define COMPUTER_H

#include "machine/particle.h"

class Computer
{
public:
    static Computer *currentComputer;

    Computer();

    static void evaluateForces(const Particle &p);
    static void evaluateSPHForces();
    static void collide();
    static void computeVectors(double dt);
    static void Euler(const Particle &p, double dt);
    static void Euler(double dt);
    static void MidPoint(double dt); // RK2? // Runge-Kutta
    static void getVolume();

    void loop();

    // FUTURE //////////////////////////////////////////////////////////////////////

    static void RKV4(double dt);

    // LEGACY //////////////////////////////////////////////////////////////////////

    void flagUp(int i, int j, int flag);
    bool flagIs(int i, int j, int flag);
};

#endif // COMPUTER_H
