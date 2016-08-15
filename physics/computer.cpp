#include "computer.h"

#include <iostream>
#include <omp.h>

class Particle;

#include "forces.h"
#include "machine/machine.h"
#include "physics/grid.h"
#include "machine/obstacle.h"
#include "machine/particle.h"
#include "machine/spring.h"
#include "physics/vector.h"
#include "util/settings.h"
#include "window/simulation_window.h" // for current frame number

Computer* Computer::currentComputer;

Computer::Computer()
{
}

void Computer::loop()
{
//#pragma omp parallel
{
    //if (SimulationWindow::frame % 10 == 0) /////////////////////////////////////
    evaluateSPHForces();
    //evaluateForces();

    computeVectors(Settings::dt);

    if (Settings::PARTICLES_REACT)
        for (unsigned int i = 0; i < Particle::flows.size(); ++i)
            for (unsigned int j = 0; j < Particle::flows.size(); ++j)
#pragma omp for
                for (unsigned int k = 0; k < Particle::flows[i].size(); ++k)
                    for (unsigned int l = 0; l < Particle::flows[j].size(); ++l)
                        if (! (k == l && i == j))
                            //Forces::collide(*Particle::flows[i][k], *Particle::flows[j][l]);
                            Forces::SPHcollide(*Particle::flows[i][k], *Particle::flows[j][l]);

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
            for (unsigned int k = 0; k < Machine::machines.size(); ++k)
                Machine::machines[k]->collide(Particle::flows[i][j]);
}
}

void Computer::evaluateForces(const Particle& p)
{
    *p.F = Vector();

#pragma omp sections
{
#pragma omp section
{
    if (Settings::FORCES_EARTH_GRAVITY)
        Forces::gravityEarth(p);
}

#pragma omp section
{
//    if (Settings::FORCES_UNIVERSAL_GRAVITY)
//        for (unsigned int i = 0; i < Particle::flows[p.parentFlow].size(); ++i)
//            Forces::universalGravitation(p, *Particle::flows[p.parentFlow][i]);
}

#pragma omp section
{
//    if (Settings::FORCES_COULOMB)
//        for (unsigned int i = 0; i < Particle::flows[p.parentFlow].size(); ++i)
//            Forces::Coulomb(p, *Particle::flows[p.parentFlow][i]);
}

#pragma omp section
{
//    //if (p.r.v[1] == -1.0f && p.v.v[1] == 0.0f) Forces.Friction(p);
}

#pragma omp section
{
    //for (Spring* s : p.springs)
    for (unsigned int i = 0; i < p.springs.size(); ++i)
    {
        Spring* s = p.springs[i];
        /*if (s.ks != 0 && s.kd != 0)*/
            Forces::Hooke(p, *s->p2, s->ks, s->d, s->kd);
    }
}
}
}

void Computer::evaluateForces()
{
    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
            evaluateForces(*Particle::flows[i][j]);
}

void Computer::evaluateSPHForces()
{
//    double m = 0.;
//    double rho = 0.;
//    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
//    {
//#pragma omp for
//        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
//        {
//            m += 1;
//            rho += Particle::flows[i][j]->rho;
//        }
//    }
//    m = rho / m / 1000;

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
        {
            //Particle::flows[i][j]->m = m;
            *Particle::flows[i][j]->F = Vector();
        }

    Grid::update();

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
            Particle::flows[i][j]->updateNeighbours();

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
        {
            Particle::flows[i][j]->updateDensity();
            Particle::flows[i][j]->updatePressure();
        }

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
        {
            Particle::flows[i][j]->computePressureForces();
            Particle::flows[i][j]->computeViscosityForces();
            Particle::flows[i][j]->computeOtherForces();
            *Particle::flows[i][j]->F /= 200;
            Particle::flows[i][j]->F->limit(10); ////////////////////////
            Particle::flows[i][j]->v->limit(10); ////////////////////////
            //Particle::flows[i][j]->F->z = 0;
        }
}

void Computer::computeVectors(const Particle& p, float dt)
{
    //MidPoint(p, dt);
    Euler(p, dt);
}
void Computer::computeVectors(float dt)
{
    //MidPoint(dt);
    Euler(dt);
}

void Computer::Euler(const Particle& p, float dt)
{
    *p.dv  = *p.F * (1 / p.m) * dt;
    *p.v  += *p.dv;
    *p.dr  = *p.v * dt;
    *p.r  += *p.dr;
    if (p.id == 1)
    {
        std::cout << *p.r << std::endl;
    }
}
void Computer::Euler(float dt)
{
    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
            Euler(*Particle::flows[i][j], dt);
}
void Computer::MidPoint(const Particle& p, float dt)
{
    Euler(p, dt * 0.5);
    evaluateForces(p);
    Euler(p, dt);
}
void Computer::MidPoint(float dt)
{
    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
            Euler(*Particle::flows[i][j], dt * 0.5);

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
            evaluateForces(*Particle::flows[i][j]);

    for (unsigned int i = 0; i < Particle::flows.size(); ++i)
#pragma omp for
        for (unsigned int j = 0; j < Particle::flows[i].size(); ++j)
            Euler(*Particle::flows[i][j], dt);
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
