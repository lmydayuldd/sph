#include "computer.h"

#include <iostream>

class Particle;

#include "forces.h"
#include "machine/machine.h"
#include "machine/obstacle.h"
#include "machine/particle.h"
#include "machine/spring.h"
#include "physics/vector.h"
#include "util/settings.h"

Computer* Computer::currentComputer;

Computer::Computer()
{
}

void Computer::loop()
{
    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
            Computer::computeVectors(*Particle::flows[i][j], Settings::dt);

    if (Settings::PARTICLES_REACT)
        for (unsigned int i = 0; i < Particle::flows.size(); ++i)
            for (unsigned int j = 0; j < Particle::flows.size(); ++j)
                for (unsigned int k = 0; k < Particle::flows[i].size(); ++k)
                    for (unsigned int l = 0; l < Particle::flows[j].size(); ++l)
                        if (! (k == l && i == j))
                            Forces::collide(*Particle::flows[i][k], *Particle::flows[j][l]);

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
            for (unsigned int k = 0; k < Machine::machines.size(); ++k)
                Machine::machines[k]->collide(Particle::flows[i][j]);
}

void Computer::evaluateForces(const Particle& p)
{
    if (Settings::FORCES_EARTH_GRAVITY)
        Forces::gravityEarth(p);

    if (Settings::FORCES_UNIVERSAL_GRAVITY)
        for (unsigned int i = 0; i < Particle::flows[p.parentFlow].size(); ++i)
            Forces::universalGravitation(p, *Particle::flows[p.parentFlow][i]);

    if (Settings::FORCES_COULOMB)
        for (unsigned int i = 0; i < Particle::flows[p.parentFlow].size(); ++i)
            Forces::Coulomb(p, *Particle::flows[p.parentFlow][i]);

    //if (p.r.v[1] == -1.0f && p.v.v[1] == 0.0f) Forces.Friction(p);

    for (Spring* s : p.springs)
        /*if (s.ks != 0 && s.kd != 0)*/
            Forces::Hooke(p, *s->p2, s->ks, s->d, s->kd);
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
