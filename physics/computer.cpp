#include "computer.h"

#include <iostream>

class Particle;

#include "forces.h"
#include "machine/flow.h"
#include "machine/machine.h"
#include "machine/obstacle.h"
#include "physics/vector.h"
#include "util/settings.h"

Computer* Computer::currentComputer;

Computer::Computer()
{
}

void Computer::loop()
{
    for (unsigned int i = 0; i < Flow::flows.size(); ++i)
        for (unsigned int j = 0; j < Flow::flows[i].particles.size(); ++j)
            Computer::computeVectors(Flow::flows[i].particles[j], Settings::dt);

    if (Settings::PARTICLES_REACT)
        for (unsigned int i = 0; i < Flow::flows.size(); ++i)
            for (unsigned int j = 0; j < Flow::flows[i].particles.size(); ++j)
                for (unsigned int k = 0; k < Flow::flows[i].particles.size(); ++k)
                    if (&Flow::flows[i].particles[j] != &Flow::flows[i].particles[k])
                        Forces::collide(Flow::flows[i].particles[j], Flow::flows[i].particles[k]);

    for (unsigned int i = 0; i < Flow::flows.size(); ++i)
        for (unsigned int j = 0; j < Flow::flows[i].particles.size(); ++j)
            for (unsigned int k = 0; k < Machine::machines.size(); ++k)
                Machine::machines[k].collide(&Flow::flows[i].particles[j]);
}

void Computer::evaluateForces(const Particle& p)
{
    if (Settings::FORCES_EARTH_GRAVITY)
        Forces::gravityEarth(p);

    if (Settings::FORCES_UNIVERSAL_GRAVITY)
        for (unsigned int i = 0; i < p.parentFlow->particles.size(); ++i)
            Forces::universalGravitation(p, p.parentFlow->particles[i]);

    if (Settings::FORCES_COULOMB)
        for (unsigned int i = 0; i < p.parentFlow->particles.size(); ++i)
            Forces::Coulomb(p, p.parentFlow->particles[i]);

    //if (p.r.v[1] == -1.0f && p.v.v[1] == 0.0f) Forces.Friction(p);
//    for (Spring s : p.springs) /*if (s.ks != 0 && s.kd != 0)*/ Forces.Hooke(p, s.p2, s.ks, s.d, s.kd);
    /*if (Settings::FORCES_HOOKE_SPRING != 0 && Settings.FORCES_HOOKE_DAMP != 0) {
        for (Particle p2 : p.parentFlow.particles)
            Forces.Hooke(p, p2, Settings.FORCES_HOOKE_SPRING, Settings.FORCES_HOOKE_DISTANCE, Settings.FORCES_HOOKE_DAMP);
    }*/
}

void Computer::computeVectors(const Particle& p, float dt)
{
    //Euler(p, dt);
    //evaluateForces(p); // ???
    MidPoint(p, dt);
}

void Computer::Euler(const Particle& p, float dt)
{
    *p.dv = *p.F * (1 / p.m) * dt;
    *p.v  = *p.v + *p.dv;
    *p.dr = *p.v * dt;
    *p.r  = *p.r + *p.dr;
}
void Computer::MidPoint(const Particle& p, float dt)
{
    Euler(p, dt * 0.5f);
    evaluateForces(p);
    Euler(p, dt);
}
void Computer::RangeKutta()
{

}
void Computer::RKV4()
{

}

void Computer::flagUp(int i, int j, int flag)
{
    i = i;
    j = j;
    flag = flag;
}
bool Computer::flagIs(int i, int j, int flag)
{
    i = i;
    j = j;
    flag = flag;
    return false;
}
