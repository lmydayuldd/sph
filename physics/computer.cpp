#include "physics/computer.h"

#include <iostream>

#include <omp.h>

extern void GPUCollideAll();
extern void GPUCollideMallocs();
extern void GPUCollideFrees();

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

Computer *Computer::currentComputer;

#ifdef COMPILER_GPP
    #define LOOP_TYPE unsigned
#elif COMPILER_MSVC
    #define LOOP_TYPE int
#endif

#define LOG_TIME(MSG) \
    if (! Settings::NO_PRINTOUT) \
        if (! Interaction::pause || SimulationWindow::key[RENDER]) \
            std::cout << (timer.diff() - formerTime)/1000000 << (MSG) << std::endl;

Computer::Computer()
{
}

void Computer::loop()
{
    Timer timer = Timer();
    long long int formerTime = 0;

    formerTime = timer.diff();
    Grid::distributeParticles();
    LOG_TIME("ms <- Distribute particles time.");

    //if (SimulationWindow::frame % 10 == 0) /////////////////////////////////////
    evaluateSPHForces();
    //evaluateForces();

    formerTime = timer.diff();
    collide();
    LOG_TIME("ms <- Collide time.");

    formerTime = timer.diff();
    computeVectors(Settings::dt);
    LOG_TIME("ms <- Compute vectors time.");

//    formerTime = timer.diff();
//    Grid::distributeParticles();
//    LOG_TIME("ms <- Distribute particles time.");

//    formerTime = timer.diff();
//    collide();
//    LOG_TIME("ms <- Collide time.");

    Computer::getVolume();
}

void Computer::evaluateForces(const Particle &p)
{
    *p.F = Vector();

    #pragma omp sections
    {
        #pragma omp section
        {
            if (Settings::FORCES_GRAVITY_EARTH) {
                Forces::gravityEarth(p);
            }
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
            //for (Spring *s : p.springs)
            for (unsigned i = 0; i < p.springs.size(); ++i)
            {
                Spring *s = p.springs[i];
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
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j)
            evaluateForces(*Particle::flows[i][j]);
}

void Computer::evaluateSPHForces()
{
    double m = 0.;
    if (Settings::CALCULATE_MASS) {
        double rho = 0.;
        int count = 0;
        for (unsigned i = 0; i < Particle::flows.size(); ++i) {
            #pragma omp for
            for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
                count += 1;
                rho += Particle::flows[i][j]->rho;
            }
        }
        m = rho / count / Settings::DESIRED_REST_DENSITY;
        std::cout << m << " <- mass" << std::endl << std::flush;
    }

    Timer timer = Timer();
    long long int formerTime = 0;

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i) {
        #pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
            if (Settings::CALCULATE_MASS) {
                Particle::flows[i][j]->m = m;
            }
            Particle::flows[i][j]->F->zero();
        }
    }
    LOG_TIME("ms <- Zero forces time.");

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i) {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
            Particle::flows[i][j]->updateNeighbours();
            Particle::flows[i][j]->isBoundary();
        }
    }
    LOG_TIME("ms <- Update neighbours & find boundaries time.");

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i) {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
            Particle::flows[i][j]->updateDensity();
            Particle::flows[i][j]->updatePressure();
        }
    }
    LOG_TIME("ms <- Update rho & p time.");

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i) {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
            Particle::flows[i][j]->computePressureForces();
            Particle::flows[i][j]->computeViscosityForces();
            Particle::flows[i][j]->computeOtherForces();
            //*Particle::flows[i][j]->F /= 1000; ////////////////////////
            Particle::flows[i][j]->F->limit(5); //////////////////////// TODO
            //Particle::flows[i][j]->v->limit(10); ////////////////////////
        }
    }
    LOG_TIME("ms <- Compute p, viscosity & other Fs time.");
}

void Computer::collide()
{
    #ifdef COMPILER_GPP
    #pragma omp parallel for \
                collapse(2) \
                if(Settings::PARALLEL_OMP)
    for (LOOP_TYPE i = 0; i < Particle::collision.size(); ++i) {
        for (LOOP_TYPE j = 0; j < Particle::collision[i].size(); ++j) {
            Particle::collision[i][j] = false;
            Particle::collisionDistance[i][j] = std::numeric_limits<double>::infinity();
        }
    }
    #elif COMPILER_MSVC
    #pragma omp parallel for if(Settings::PARALLEL_OMP)
    for (LOOP_TYPE n = 0; n < Particle::collision.size() * Particle::collision.size(); ++n) {
        LOOP_TYPE i = n / Particle::collision.size();
        LOOP_TYPE j = n % Particle::collision.size();
        Particle::collision[i][j] = false;
        Particle::collisionDistance[i][j] = std::numeric_limits<double>::infinity();
    }
    #endif

    #ifdef COMPILER_GPP
    #pragma omp parallel for \
                collapse(2) \
                if(Settings::PARALLEL_OMP)
    for (LOOP_TYPE i = 0; i < Particle::flows.size(); ++i)
         for (LOOP_TYPE j = 0; j < Particle::flows.size(); ++j)
             *Particle::flows[i][j]->dt_left = Settings::dt; // TODO 1. or Settings::dt ???
    #elif COMPILER_MSVC
    #pragma omp parallel if(Settings::PARALLEL_OMP)
    for (LOOP_TYPE n = 0; n < Particle::flows.size() * Particle::flows.size(); ++n) {
        LOOP_TYPE i = n / Particle::flows.size();
        LOOP_TYPE j = n % Particle::flows.size();
        *Particle::flows[i][j]->dt_left = Settings::dt; // TODO 1. or Settings::dt ???
    }
    #endif

//////////////////////////////////////////////////////////////

    if (Settings::PARTICLES_REACT) {
        int c = 1;

#ifdef COMPILER_MSVC
        if (Settings::PARALLEL_GPU) {
            c = 0;
        }

        if (c == 0) {
            GPUCollideMallocs();
            GPUCollideAll();
            //GPUCollideFrees();
        }
#endif

        if (c == 1)
        for (LOOP_TYPE i = 0; i < Particle::flows.size(); ++i) {
            for (LOOP_TYPE j = 0; j < Particle::flows.size(); ++j) {
#ifdef COMPILER_GPP
                #pragma omp parallel for \
                            if(Settings::PARALLEL_OMP) \
                            schedule(guided, 4)// \
                            //collapse(2)
                for (LOOP_TYPE k = 0; k < Particle::flows[i].size(); ++k) {
                    for (LOOP_TYPE l = k; l < Particle::flows[j].size(); ++l) {
                        if (! (k == l && i == j)) {
                            //if (! Particle::flows[i][k]->didCollide)
                                Forces::collide(*Particle::flows[i][k],
                                                *Particle::flows[j][l]);
                        }
                    }
                }
#elif COMPILER_MSVC
//                #pragma omp parallel for \
//                            if(Settings::PARALLEL_OMP)
//                for (LOOP_TYPE n = 0; n < Particle::flows[i].size() * Particle::flows[j].size(); ++n) {
//                    LOOP_TYPE k = 0;
//                    LOOP_TYPE l = 0;
//                    for (LOOP_TYPE x = n, y = Particle::flows[j].size(); ; ) {
//                        if (x >= y) { x -= y; --y; ++k; }
//                        else        { l = x; break; }
//                    }
//                    if (! (k == l && i == j)) {
//                        //if (! Particle::flows[i][k]->didCollide)
//                            Forces::collide(*Particle::flows[i][k],
//                                            *Particle::flows[j][l]);
//                    }
//                }
                #pragma omp parallel for \
                            if(Settings::PARALLEL_OMP) \
                            schedule(guided, 4)
                for (LOOP_TYPE k = 0; k < Particle::flows[i].size(); ++k) {
                    for (LOOP_TYPE l = k; l < Particle::flows[j].size(); ++l) {
                        if (! (k == l && i == j)) {
                            //if (! Particle::flows[i][k]->didCollide)
                                Forces::collide(*Particle::flows[i][k],
                                                *Particle::flows[j][l]);
                        }
                    }
                }
#endif
            }
        }

        if (c == 2)
        for (unsigned i = 0; i < Particle::flows.size(); ++i) {
            //#pragma omp parallel for collapse(2) schedule(guided) \
            //                     if(Settings::PARALLEL_OMP)
            for (unsigned j = 0; j < Particle::flows[i].size(); ++j) {
                for (unsigned k = 0; k < Particle::flows[i][j]->neighbours->size(); ++k) {
                    Forces::collide(*Particle::flows[i][j],
                                    *Particle::flows[i][j]->neighbours->at(k));
                }
            }
        }

        if (c == 3)
        for (unsigned i = 0; i < Particle::flows.size(); ++i) {
            for (unsigned j = 0; j < Particle::flows[i].size(); ++j) {
    //            if (! Particle::flows[i][j]->didCollide)
                    Forces::collide(*Particle::flows[i][j],
                                    //Particle::flows[i][j]->neighbours);
                                    // UNSAFE!!! address of vector cell!!! TODO //////
                                    &Particle::flows[Particle::flows[i][j]->parentFlow]);
            }
        }
    }

//////////////////////////////////////////////////////////////

    for (unsigned i = 0; i < Particle::flows.size(); ++i)
    {
        #ifdef COMPILER_GPP
        #pragma omp parallel for collapse(2) if(Settings::PARALLEL_OMP)
        for (unsigned j = 0; j < Particle::flows[i].size(); ++j)
            for (unsigned k = 0; k < Machine::machines.size(); ++k)
                Machine::machines[k]->collide(Particle::flows[i][j]);
        #elif COMPILER_MSVC
        #pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE n = 0; n < Particle::flows[i].size() * Machine::machines.size(); ++n)
        {
            LOOP_TYPE j = n / Machine::machines.size();
            LOOP_TYPE k = n % Machine::machines.size();
            Machine::machines[k]->collide(Particle::flows[i][j]);
        }
        #endif
    }

    for (unsigned i = 0; i < Particle::flows.size(); ++i)
    {
        #pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j)
        {
            *Particle::flows[i][j]->didCollide = false;
        }
    }
}

void Computer::computeVectors(const Particle &p, float dt)
{
    //MidPoint(p, dt);
    Euler(p, dt);
}
void Computer::computeVectors(float dt)
{
    //MidPoint(dt);
    Euler(dt);
}

void Computer::Euler(const Particle &p, float dt)
{
    if (! p.isStationary)
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
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j)
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
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j)
            Euler(*Particle::flows[i][j], dt * 0.5);

    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j)
            evaluateForces(*Particle::flows[i][j]);

    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j)
            Euler(*Particle::flows[i][j], dt);
}

void Computer::getVolume()
{
    int volume = 0;

    #ifdef COMPILER_GPP
    #pragma omp parallel for \
                if(Settings::PARALLEL_OMP) \
                collapse(3) \
                reduction(+:volume)
    for (LOOP_TYPE i = 0; i < Grid::cell_count; ++i)
        for (LOOP_TYPE j = 0; j < Grid::cell_count; ++j)
            for (LOOP_TYPE k = 0; k < Grid::cell_count; ++k)
                if (Grid::grid[i][j][k].size() > 0)
                    ++volume;
    #elif COMPILER_MSVC
    #pragma omp parallel for if(Settings::PARALLEL_OMP)
    for (LOOP_TYPE n = 0; n < Grid::cell_count * Grid::cell_count * Grid::cell_count; ++n)
    {
        int i =  n / (Grid::cell_count * Grid::cell_count);
        int j = (n % (Grid::cell_count * Grid::cell_count)) / Grid::cell_count;
        int k = (n % (Grid::cell_count * Grid::cell_count)) % Grid::cell_count;
        if (Grid::grid[i][j][k].size() > 0) {
            ++volume;
        }
    }
    #endif

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
