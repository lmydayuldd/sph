#include "computer.h"

#include <iostream>
#include <omp.h>

class Particle;

#include "control/interaction.h"
#include "machine/machine.h"
#include "machine/obstacle.h"
#include "machine/particle.h"
#include "machine/spring.h"
#include "physics/forces.h"
#include "physics/grid.h"
#include "physics/vector.h"
#include "util/settings.h"
#include "util/timer.h"
#include "window/simulation_window.h" // for current frame number

Computer* Computer::currentComputer;

Computer::Computer()
{
}

void Computer::loop()
{
    Timer timer = Timer();
    long long int formerTime = 0;

    formerTime = timer.diff();
    Grid::distributeParticles();
    if (! Interaction::pause || SimulationWindow::key[RENDER])
        std::cout << (timer.diff() - formerTime)/1000000
                  << "ms <- Distribute particles time." << std::endl;

    //if (SimulationWindow::frame % 10 == 0) /////////////////////////////////////
    evaluateSPHForces();
    //evaluateForces();

    formerTime = timer.diff();
    collide();
    if (! Interaction::pause || SimulationWindow::key[RENDER])
        std::cout << round((timer.diff() - formerTime)/1000000.0)
                  << "ms <- Collide time." << std::endl;

    formerTime = timer.diff();
    computeVectors(Settings::dt);
    if (! Interaction::pause || SimulationWindow::key[RENDER])
        std::cout << round((timer.diff() - formerTime)/1000000.0)
                  << "ms <- Compute vectors time." << std::endl;

//    Grid::distributeParticles();
//    if (! Interaction::pause || SimulationWindow::key[RENDER])
//        std::cout << (timer.diff() - formerTime)/1000000
//                  << "ms <- Distribute particles time." << std::endl;

//    formerTime = timer.diff();
//    collide();
//    if (! Interaction::pause || SimulationWindow::key[RENDER])
//        std::cout << round((timer.diff() - formerTime)/1000000.0)
//                  << "ms <- Collide time." << std::endl;

    Computer::getVolume();
}

void Computer::evaluateForces(const Particle& p)
{
    *p.F = Vector();

    #pragma omp sections
    {
        #pragma omp section
        {
            if (Settings::FORCES_GRAVITY_EARTH)
                Forces::gravityEarth(p);
        }

        #pragma omp section
        {
        //    if (Settings::FORCES_UNIVERSAL_GRAVITY)
        //        for (unsigned i = 0; i < Particle::flows[p.parentFlow].size(); ++i)
        //            Forces::universalGravitation(p, *Particle::flows[p.parentFlow][i]);
        }

        #pragma omp section
        {
        //    if (Settings::FORCES_COULOMB)
        //        for (unsigned i = 0; i < Particle::flows[p.parentFlow].size(); ++i)
        //            Forces::Coulomb(p, *Particle::flows[p.parentFlow][i]);
        }

        #pragma omp section
        {
        //    //if (p.r.v[1] == -1.0f && p.v.v[1] == 0.0f) Forces.Friction(p);
        }

        #pragma omp section
        {
            //for (Spring* s : p.springs)
            for (unsigned i = 0; i < p.springs.size(); ++i)
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
    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
            evaluateForces(*Particle::flows[i][j]);
}

void Computer::evaluateSPHForces()
{
    // TODO
//    double m = 0.;
//    double rho = 0.;
//    int count = 0;
//    for (unsigned i = 0; i < Particle::flows.size(); ++i)
//    {
//#pragma omp for
//        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
//        {
//            count += 1;
//            rho += Particle::flows[i][j]->rho;
//        }
//    }
//    m = rho / count / Settings::DESIRED_REST_DENSITY;
//    std::cout << m << " <- mass" << std::endl << std::flush;

    Timer timer = Timer();
    long long int formerTime = 0;

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i)
    {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
        {
            //Particle::flows[i][j]->m = m;
            Particle::flows[i][j]->F->zero();
        }
    }
    if (! Interaction::pause || SimulationWindow::key[RENDER])
        std::cout << round((timer.diff() - formerTime)/1000000.0)
                  << "ms <- Zero forces time." << std::endl;

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i)
    {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
        {
            Particle::flows[i][j]->updateNeighbours();
            Particle::flows[i][j]->isBoundary();
        }
    }
    if (! Interaction::pause || SimulationWindow::key[RENDER])
        std::cout << round((timer.diff() - formerTime)/1000000.0)
                  << "ms <- Update neighbours & find boundaries time." << std::endl;

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i)
    {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
        {
            Particle::flows[i][j]->updateDensity();
            Particle::flows[i][j]->updatePressure();
        }
    }
    if (! Interaction::pause || SimulationWindow::key[RENDER])
        std::cout << round((timer.diff() - formerTime)/1000000.0)
                  << "ms <- Update rho & p time." << std::endl;

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i)
    {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
        {
            Particle::flows[i][j]->computePressureForces();
            Particle::flows[i][j]->computeViscosityForces();
            Particle::flows[i][j]->computeOtherForces();
            //*Particle::flows[i][j]->F /= 1000; ////////////////////////
            Particle::flows[i][j]->F->limit(5); //////////////////////// TODO
            //Particle::flows[i][j]->v->limit(10); ////////////////////////
        }
    }
    if (! Interaction::pause || SimulationWindow::key[RENDER])
        std::cout << round((timer.diff() - formerTime)/1000000.)
                  << "ms <- Compute p, viscosity & other Fs time." << std::endl;
}

void Computer::collide()
{
    for (unsigned i = 0; i < Particle::collision.size(); ++i)
        for (unsigned j = 0; j < Particle::collision[i].size(); ++j)
        {
            Particle::collision[i][j] = false;
            Particle::collisionDistance[i][j] = std::numeric_limits<double>::infinity();
        }

//#pragma omp parallel for \
//            collapse(2) \
//            if(Settings::PARALLEL_OMP)
    for (unsigned i = 0; i < Particle::flows.size(); ++i)
         for (unsigned j = 0; j < Particle::flows.size(); ++j)
             *Particle::flows[i][j]->dt_left = Settings::dt; // TODO 1. or Settings::dt ???

//////////////////////////////////////////////////////////////

    int c = 1;

    if (c == 1)
    if (Settings::PARTICLES_REACT)
        for (unsigned i = 0; i < Particle::flows.size(); ++i)
            for (unsigned j = 0; j < Particle::flows.size(); ++j)
                #pragma omp parallel for \
                            schedule(guided, 4) \
                            if(Settings::PARALLEL_OMP)
                            //collapse(2)
                for (unsigned k = 0; k < Particle::flows[i].size(); ++k)
                    for (unsigned l = k; l < Particle::flows[j].size(); ++l)
                        if (! (k == l && i == j))
                            //if (! Particle::flows[i][k]->didCollide)
                                Forces::collide(*Particle::flows[i][k],
                                                *Particle::flows[j][l]);

    if (c == 2)
    if (Settings::PARTICLES_REACT)
        for (unsigned i = 0; i < Particle::flows.size(); ++i)
            //#pragma omp parallel for collapse(2) schedule(guided) \
            //                     if(Settings::PARALLEL_OMP)
            for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
                for (unsigned k = 0; k < Particle::flows[i][j]->neighbours->size(); ++k)
                    Forces::collide(*Particle::flows[i][j],
                                    *Particle::flows[i][j]->neighbours->at(k));

    if (c == 3)
    if (Settings::PARTICLES_REACT)
        for (unsigned i = 0; i < Particle::flows.size(); ++i)
            for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
    //            if (! Particle::flows[i][j]->didCollide)
                    Forces::collide(*Particle::flows[i][j],
                                    //Particle::flows[i][j]->neighbours);
                                    // UNSAFE!!! address of vector cell!!! TODO //////
                                    &Particle::flows[Particle::flows[i][j]->parentFlow]);

//////////////////////////////////////////////////////////////

    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for collapse(2) if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
            for (unsigned k = 0; k < Machine::machines.size(); ++k)
                Machine::machines[k]->collide(Particle::flows[i][j]);

    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
            *Particle::flows[i][j]->didCollide = false;
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
    if (! p.stationary)
    {
        *p.r_former = *p.r;
        *p.v_former = *p.v;

        *p.dv  = *p.F * (1. / p.m) * dt;
        *p.v  += *p.dv;
        *p.dr  = *p.v * dt;
        //*p.dr = p.dr->normal() * Settings::PARTICLE_MAX_DR;
        *p.r  += *p.dr;

    //    if (p.id == 1)
    //    {
    //        std::cout << *p.r << std::endl;
    //    }
    }
}
void Computer::Euler(float dt)
{
    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
            Euler(*Particle::flows[i][j], dt);
}
void Computer::MidPoint(const Particle& p, float dt) // RK2?
{
    Euler(p, dt * 0.5);
    evaluateForces(p);
    Euler(p, dt);
}
void Computer::MidPoint(float dt) // RK2?
{
    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
            Euler(*Particle::flows[i][j], dt * 0.5);

    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
            evaluateForces(*Particle::flows[i][j]);

    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
            Euler(*Particle::flows[i][j], dt);
}

void Computer::getVolume()
{
    int volume = 0;
#pragma omp parallel for \
            collapse(3) \
            reduction(+:volume) \
            if(Settings::PARALLEL_OMP)
    for (unsigned i = 0; i < Grid::cell_count; ++i)
        for (unsigned j = 0; j < Grid::cell_count; ++j)
            for (unsigned k = 0; k < Grid::cell_count; ++k)
                if (Grid::grid[i][j][k].size() > 0)
                    ++volume;
    std::cout << volume << "cbs <- Volume." << std::endl << std::flush;
}

// FUTURE //////////////////////////////////////////////////////////////////////

void Computer::RKV4()
{

}

// LEGACY //////////////////////////////////////////////////////////////////////

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
