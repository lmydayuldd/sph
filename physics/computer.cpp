#include "physics/computer.h"

class Particle;

#include "control/interaction.h"
#include "machine/obstacle.h"
#include "machine/particle.h"
#include "machine/spring.h"
#include "physics/forces.h"
#include "physics/grid.h"
#include "physics/vector.h"
#include "util/settings.h"
#include "util/timer.h"
#include "window/simulation_window.h" // for current frame number

#include <iostream>

#include <omp.h>

#include "util/macros.h"

extern void GPUCollideAll();
extern void GPUCollideMallocs();
extern void GPUCollideFrees();

Computer *Computer::currentComputer;

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

    formerTime = timer.diff();
    collide();
    LOG_TIME("ms <- Collide time.");

    // TODO
//    for (unsigned i = 0; i < Particle::flows[0].size(); ++i) {
//        Particle::flows[0][i]->F->cut(Settings::PARTICLE_MAX_DR);
//    }

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
//    if      (Settings::COLOR_BY == VELOCITY)  Particle::evaluateMaximalVelocity();
//    else if (Settings::COLOR_BY == PRESSURE)  Particle::evaluateMaximalPressure();
//    else if (Settings::COLOR_BY == VISCOSITY) Particle::evaluateMaximalViscosity();
//    else if (Settings::COLOR_BY == TENSION)   Particle::evaluateMaximalTension();
    Particle::evaluateMaximalVelocity();
    Particle::evaluateMaximalPressure();
    Particle::evaluateMaximalViscosity();
    Particle::evaluateMaximalTension();
    std::cout << Particle::v_avg_norm << " <- v_avg " << std::endl << std::flush;
    std::cout << Particle::F_P_avg_norm << " <- F_P_avg" << std::endl << std::flush;
    std::cout << Particle::F_visc_avg_norm << " <- F_visc_avg" << std::endl << std::flush;
    std::cout << Particle::F_tens_avg_norm << " <- F_tens_avg" << std::endl << std::flush;
    for (unsigned i = 0; i < Particle::flows.size(); ++i) {
        #pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
            if (Settings::CALCULATE_MASS) {
                Particle::flows[i][j]->m = m;
            }
            // TODO
            //Particle::flows[i][j]->m = Particle::flows[i][j]->rho;
            //Particle::flows[i][j]->m *= Particle::flows[i][j]->rho / Settings::DESIRED_REST_DENSITY;
            Particle::flows[i][j]->F->zero();
        }
    }
    LOG_TIME("ms <- Zero forces and evaluate max/avg v/F_P/F_visc/F_surf_tens time.");

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i) {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
            Particle::flows[i][j]->updateNeighbours();
        }
    }
    LOG_TIME("ms <- Update neighbours time.");

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i) {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
            Particle::flows[i][j]->updateDensity();
            Particle::flows[i][j]->isBoundary();
            Particle::flows[i][j]->updatePressure();
        }
    }
    LOG_TIME("ms <- Update rho & p and find boundaries time.");

    formerTime = timer.diff();
    for (unsigned i = 0; i < Particle::flows.size(); ++i) {
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
            if (Settings::GRANULAR_OR_LIQUID) {
                Particle::flows[i][j]->computePressureForces();
                Particle::flows[i][j]->computeViscosityForces();
                Particle::flows[i][j]->computeSurfaceTensionForces(); // TODO
            }
            Particle::flows[i][j]->computeOtherForces();
            Particle::flows[i][j]->F->cut(Settings::FORCES_LIMIT); // TODO
//            if (Particle::flows[i][j]->id == 500) {
//                //std::cout << "F_p500 " << *Particle::flows[i][j]->F << std::endl << std::flush;
//                //std::cout << "rho_p500 " << Particle::flows[i][j]->rho << std::endl << std::flush;
//                //std::cout << "P_p500 " << Particle::flows[i][j]->pressure << std::endl << std::flush;
//            }
        }
    }
    LOG_TIME("ms <- Compute p, viscosity & other Fs time.");

//#pragma omp section
//    {
//        //for (Spring *s : p.springs)
//        for (unsigned i = 0; i < p.springs.size(); ++i)
//        {
//            Spring *s = p.springs[i];
//            /*if (s.ks != 0 && s.kd != 0)*/
//                Forces::Hooke(p, *s->p2, s->ks, s->d, s->kd);
//        }
//    }
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

        if (Settings::COLLIDE_NEIGHBOURS_ONLY)
        {
            c = 2;
        }

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
#ifdef COMPILER_GPP
        for (LOOP_TYPE i = 0; i < Particle::flows.size(); ++i) {
//            #pragma omp parallel for collapse(2) schedule(guided) \
//                                 if(Settings::PARALLEL_OMP)
            for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
                for (LOOP_TYPE k = 0; k < Particle::flows[i][j]->neighbours->size(); ++k) {
                    Forces::collide(*Particle::flows[i][j],
                                    *Particle::flows[i][j]->neighbours->at(k));
                }
            }
        }
#elif COMPILER_MSVC
        for (LOOP_TYPE i = 0; i < Particle::flows.size(); ++i) {
            #pragma omp parallel for if(Settings::PARALLEL_OMP)
            for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j) {
                for (LOOP_TYPE k = 0; k < Particle::flows[i][j]->neighbours->size(); ++k) {
//                    if (! Particle::collision[Particle::flows[i][j]->id-1][Particle::flows[i][j]->neighbours->at(k)->id-1]) {
                        Forces::collide(*Particle::flows[i][j],
                                        *Particle::flows[i][j]->neighbours->at(k));
//                    }
                }
            }
        }
#endif

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

void Computer::computeVectors(double dt)
{
    switch (Settings::INTEGRATION_SCHEME)
    {
        case EULER         :               break;
        case REVERSE_EULER : Euler(dt);    break;
        case LEAPFROG      :               break;
        case MIDPOINT      : MidPoint(dt); break;
        case RK4           : RKV4(dt);     break;
        default :
            std::cout << "Switch failure at computeVectors()!" << std::endl << std::flush;
            exit(0);
    }
}

void Computer::Euler(const Particle &p, double dt)
{
    if (! p.isStationary)
    {
        *p.r_former = *p.r;
        *p.v_former = *p.v;

//        *p.dv  = *p.F * (1. / p.m / (p.rho / Settings::DESIRED_REST_DENSITY)) * dt;
        *p.dv  = *p.F * (1. / p.m) * dt;
        *p.v  += *p.dv;
        *p.dr  = *p.v * dt;
        if (p.dr->norm() > Settings::PARTICLE_MAX_DR)
            *p.dr = p.dr->normal() * Settings::PARTICLE_MAX_DR;
        *p.r  += *p.dr;
    }
}
void Computer::Euler(double dt)
{
    for (unsigned i = 0; i < Particle::flows.size(); ++i)
#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (LOOP_TYPE j = 0; j < Particle::flows[i].size(); ++j)
            Euler(*Particle::flows[i][j], dt);
}
void Computer::MidPoint(double dt) // RK2
{
    Euler(dt * 0.5);
    evaluateSPHForces();
    Euler(dt);
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

void Computer::RKV4(double dt)
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
