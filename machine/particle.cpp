#include "machine/particle.h"

#include <iostream>
#include <random>

#include <omp.h>
#include "mpi.h"

#include "gl/matrices.h"
#include "machine/spring.h"
#include "physics/forces.h"
#include "physics/grid.h"
#include "physics/vector.h"
#include "shape/dot.h"
#include "shape/line.h"
#include "shape/sphere.h"
#include "util/constants.h"
#include "util/map.h"
#include "util/settings.h"

//#define LOOP_TYPE unsigned
#define LOOP_TYPE int

using namespace std;

std::vector<std::vector<Particle*>> Particle::flows;
int Particle::count = 0;
std::vector<std::vector<bool>> Particle::collision;
std::vector<std::vector<double>> Particle::collisionDistance;

Particle::Particle()
{
}

Particle::~Particle()
{
    //delete parentFlow;
    delete r;
    delete v;
    delete r_former;
    delete v_former;
    delete a;
    delete dr;
    delete dv;
    delete da;
    delete F;
    for (unsigned i = 0; i < springs.size(); ++i) {
        delete springs[i];
    }
    delete didCollide;
    //delete cube;
    delete neighbours;
}

Particle::Particle(int parentFlow)
    : parentFlow(parentFlow),
      id(++count),
      neighbours(nullptr),
      cell(std::vector<unsigned>{ std::numeric_limits<unsigned>::max(),
                                      std::numeric_limits<unsigned>::max(),
                                      std::numeric_limits<unsigned>::max() }),
      cube(nullptr),
      m(Settings::PARTICLE_MASS),
      rho(Settings::DESIRED_REST_DENSITY),
      charge(Settings::FORCES_COULOMB),
      temperature(Settings::TEMPERATURE),
      viscosity(Settings::VISCOSITY),
      pressure(Settings::PRESSURE),
      smoothing_length(Settings::SMOOTHING_LENGTH),
      radius(Settings::PARTICLE_RADIUS),
      stationary(false),
      didCollide(new bool(false)),
      boundary(false),
      color {0.5, 0, 0},
      dt_left(new double(Settings::dt)),
      a(new Vector()),
      dr(new Vector()),
      dv(new Vector()),
      da(new Vector()),
      F(new Vector()),
      r(new Vector()),
      v(new Vector())
{
    switch (Settings::MAP_SETUP)
    {
        case DAM_BREAK_NON_MAP :
            r = new Vector(
                (-Settings::ARENA_DIAMETER/2 + radius) + fmod(id-1, 20) * Settings::PARTICLES_INIT_DIST,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
                (-Settings::ARENA_DIAMETER/2 + radius) + (id-1)/20 * Settings::PARTICLES_INIT_DIST,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
                0//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5)
            );
        break;
        case RANDOM_NON_MAP :
            r = new Vector(
                (-Settings::ARENA_DIAMETER/2 + radius) + (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * Settings::ARENA_DIAMETER,
                (-Settings::ARENA_DIAMETER/2 + radius) + (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * Settings::ARENA_DIAMETER,
                0//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5)
            );
        break;
    }

//    v = new Vector(
//        (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%30),
//        (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%30),
//        (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%30)
//    );
    r_former = new Vector(*r);
    v_former = new Vector(*v);

    linkView(SPHERE);
}

void Particle::createView()
{
    if (Settings::DOT_OR_SPHERE) {
        Sphere sphere(1.0, color);
        this->form = sphere.form;
    }
    else
    {
        Dot dot(color);
        this->form = dot.form;
    }
}

void Particle::springify(Particle* p2, float ks, float d, float kd)
{
    springs.push_back(new Spring(p2, ks, d, kd));
}
void Particle::springifyMutual(Particle* p2, float ks, float d, float kd)
{
    springs.push_back(new Spring(p2, ks, d, kd));
    p2->springs.push_back(new Spring(this, ks, d, kd)); ////////////////////////
}

double Particle::v_max()
{
    double v_max = - std::numeric_limits<double>::infinity();
    for (unsigned i = 0; i < Particle::flows[parentFlow].size(); ++i)
    {
        if (Particle::flows[parentFlow][i]->v->norm() > v_max)
        {
            v_max = Particle::flows[parentFlow][i]->v->norm();
        }
    }
    return v_max;
}

void Particle::updateNeighbours()
{
//    if (stationary)
//        neighbours = new std::vector<Particle*>();
//        return; ////////////////////////////////////////////////////////

    // naive // incomputable for large particle count
    if (Settings::NEIGHBOUR_CHOICE.compare("ALL") == 0) {
        unsigned neighbour_count = Settings::NEIGHBOUR_COUNT;
        double neighbour_range = std::numeric_limits<double>::infinity();//2 * smoothing_length;

        unsigned particle_count = Particle::flows[this->parentFlow].size();
        if (neighbours == nullptr)
        {
            neighbours = new std::vector<Particle*>();
        }
        else
        {
            neighbours->clear();
        }

        double last_min_distance = 0;
        for (unsigned i = 0; i < neighbour_count && i < particle_count; ++i)
        {
            double min_distance = std::numeric_limits<double>::infinity();
            unsigned min_index = std::numeric_limits<unsigned>::max();
            for (unsigned j = 0; j < particle_count; ++j)
            {
                if (this != Particle::flows[parentFlow][j])
                {
                    double j_distance
                            = r->distance(*Particle::flows[parentFlow][j]->r);
                    if (j_distance < min_distance && j_distance <= neighbour_range)
                    {
                        if (j_distance > 0 && j_distance > last_min_distance)
                        {
                            min_distance = j_distance;
                            min_index = j;
                        }
                    }
                }
            }
            if (min_index != std::numeric_limits<unsigned>::max())
            {
                neighbours->push_back(Particle::flows[parentFlow][min_index]);
            }
            last_min_distance = min_distance;
        }
    }

    // grid-supported
    else if (Settings::NEIGHBOUR_CHOICE.compare("GRID") == 0) {
        if (neighbours == nullptr)
        {
            neighbours = new std::vector<Particle*>();
        }
        else
        {
            neighbours->clear();
        }

        // TODO
        // shouldn't all particles be checked, not all grid cells?
        // if cells are checked first, then the more there are cells
        // (the bigger the arena is), the more checks there are
        // (of mostly empty cells what's more),
        // but if particles were checked first, to find in which
        // cell they are, and then find neighbhours among
        // particles of this cell, than it would be much fast
        // (? having brought no disadvantages at the same time ?)
        Particle *p = nullptr;
#pragma omp parallel for \
            if(Settings::PARALLEL_OMP)
//            collapse(3)
//            firstprivate(p) // TODO
        for (LOOP_TYPE i = std::max<int>((int)cell[0] - 1, 0);
             i <= std::min<unsigned>(cell[0] + 1, Grid::cell_count - 1);
             ++i) {
            for (LOOP_TYPE j = std::max<int>((int)cell[1] - 1, 0);
                 j <= std::min<unsigned>(cell[1] + 1, Grid::cell_count - 1);
                 ++j) {
                for (LOOP_TYPE k = std::max<int>((int)cell[2] - 1, 0);
                     k <= std::min<unsigned>(cell[2] + 1, Grid::cell_count - 1);
                     ++k) {
                    for (LOOP_TYPE l = 0;
                         l < Grid::grid[i][j][k].size();
                         ++l)
                    {
                        p = Grid::grid[i][j][k][l];
                        if (this != p)
                        {
                            if (Settings::NEIGHBOUR_LMT_BY_CNT
                                && neighbours->size() > Settings::NEIGHBOUR_COUNT)
                            {
                                continue; /////////////////////////////////////////////////////////////////////////////
                            }
                            if (Settings::NEIGHBOUR_LMT_BY_DST
                                && (r->distance(*p->r) > Settings::NEIGHBOUR_RANGE))
                            {
                                continue; /////////////////////////////////////////////////////////////////////////////
                            }
//                            if (id == 20) {
//                                float f[3] = {1.0f, 0, 0};
//                                recolor(f);
//                            }
//#pragma omp critical
                            neighbours->push_back(p);
                        }
                    }
                }
            }
        }
        if (id == 50) {
            std::cout << neighbours->size()
                      << "<- Particle 50 neighbour count." << std::endl << std::flush;
        }
        if (id == 20) {
            std::cout << neighbours->size()
                      << "<- Particle 20 neighbour count." << std::endl << std::flush;
        }
    }

    // octree-supported
    else if (Settings::NEIGHBOUR_CHOICE.compare("OCTREE") == 0) {

    }

    // printout
//#pragma omp critical
//    {
//        std::cout << id << " has " << neighbours->size() << " neighs: " << std::flush;
//        for (unsigned i = 0; i < neighbours->size(); ++i)
//        {
//            std::cout << (*neighbours)[i]->id << " " << std::flush;
//        }
//        std::cout << std::endl << std::flush;
//    }
}

void Particle::isBoundary()
{
    if (neighbours->size() < 2)
    {
        boundary = true;
    }
    else {
        boundary = false;
    }
}

/**
 * @brief Cubic spline smooth function
 */
double Particle::kernelFunction(Particle* p2)
{
    double h = smoothing_length;

    double C_h = 0.;
    switch (Settings::KERNEL_DIM)
    {
        case 1: C_h = 1.  / ( 6        * h);         break;
        case 2: C_h = 15. / (14 * M_PI * pow(h, 2)); break;
        case 3: C_h = 1.  / ( 4 * M_PI * pow(h, 3)); break;
    }

    double kernel = 0.;
    double q = r->distance(*p2->r) / h;
    if      (q >= 0. && q < 1.) kernel = C_h * (pow(2-q, 3) - 4 * pow(1-q, 3));
    else if (q >= 1. && q < 2.) kernel = C_h *  pow(2-q, 3);
    else if (q >= 2.)           kernel = 0.;

    //std::cout << "distance " << r->distance(*p2->r) << std::endl << std::flush;
    //std::cout << "QQQQQQQQQQ " << q << std::endl << std::flush;
    //std::cout << "Kernel " << kernel << std::endl << std::flush;
    return kernel;
}

Vector Particle::kernelFunctionGradient(Particle* p2)
{
    double h = smoothing_length;

    double C_h = 0.;
    switch (Settings::KERNEL_DIM)
    {
        case 1: C_h =  -1. / (6        * pow(h, 2)); break;
        case 2: C_h = -15. / (7 * M_PI * pow(h, 3)); break;
        case 3: C_h =  -3. / (4 * M_PI * pow(h, 4)); break;
    }

    double grad_W = 0;
    double q = r->distance(*p2->r) / h;
//    if      (q >= 0 && q < 1) grad_W = C_h * (-0.75 * (pow(q, 2) - 4*q + 4));
//    else if (q >= 1 && q < 2) grad_W = C_h * (-pow(q, 3) + 2*pow(q, 2) - 4*q + 8);
//    else if (q >= 2)          grad_W = 0;
    if      (q >= 0. && q < 1.) grad_W = C_h *  q * (9*q - 4);
    else if (q >= 1. && q < 2.) grad_W = C_h * (q * (4 - 3*q) - 4);
    else if (q >= 2.)           grad_W = 0;

    //std::cout << "Kernel Function gradient " << grad_W << std::endl << std::flush;
    return (*p2->r - *r) / r->distance(*p2->r) * grad_W;
}

void Particle::updateDensity()
{
    ///////////// TODO, why save density if no neighbours?
    if (neighbours->size() > 0)
    {
        rho = 0;
//        double rho_local = 0.;
// only slows down
//#pragma omp parallel for \
//            reduction(+:rho_local) \
//            if(Settings::PARALLEL_OMP)
        for (unsigned i = 0; i < neighbours->size(); ++i)
        {
            rho += (*neighbours)[i]->m * kernelFunction((*neighbours)[i]);
//            rho_local += (*neighbours)[i]->m * kernelFunction((*neighbours)[i]);
        }
//        rho = rho_local;
    }
    //std::cout << "Density rho " << rho << std::endl << std::flush;
}

void Particle::updatePressure()
{
    double k     = Settings::STIFFNESS_CONSTANT;
    double rho_0 = Settings::DESIRED_REST_DENSITY;

    pressure = k * (pow(rho/rho_0, 7) - 1);
    //std::cout << "Pressure " << pressure << std::endl << std::flush;
}

void Particle::computePressureForces()
{
    Vector gradient_pressure = Vector(); // approximated
    Particle *n = nullptr;
//#pragma omp parallel for \
//            shared(gradient_pressure) \
//            if(Settings::PARALLEL_OMP)
    for (unsigned i = 0; i < neighbours->size(); ++i)
    {
        n = (*neighbours)[i];
//#pragma omp critical
        gradient_pressure +=
            kernelFunctionGradient(n)
            * (pressure/pow(rho, 2) + n->pressure/pow(n->rho, 2))
            * n->m;
    }
    gradient_pressure *= rho;

    Vector F_p = Vector();
    F_p = gradient_pressure * (- m/rho);
    *F += F_p;
    //std::cout << "F_pressure " << F_p << std::endl << std::flush;
}

void Particle::computeViscosityForces()
{
    Vector velocity_gradient_squared = Vector(); // approximated
    Particle *n = nullptr;
//#pragma omp parallel for \
//            shared(velocity_gradient_squared) \
//            if(Settings::PARALLEL_OMP)
    for (unsigned i = 0; i < neighbours->size(); ++i)
    {
        n = (*neighbours)[i];
//#pragma omp critical
        velocity_gradient_squared +=
            (*r - *n->r)
            * kernelFunctionGradient(n)
            * (*v - *n->v)
            * (n->m / n->rho)
            / ((*r - *n->r).dot(*r - *n->r)
               + 0.01 * pow(smoothing_length, 2));
        //std::cout << "v - n->v" << (v - n->v) << std::endl << std::flush;
    }
    velocity_gradient_squared *= 2;
    //std::cout << "velocity_gradient_squared " << velocity_gradient_squared << std::endl << std::flush;

    Vector F_viscosity = Vector();
    F_viscosity = velocity_gradient_squared * viscosity * m;
    *F += F_viscosity;
//#pragma omp critical
//    {
//        std::cout << "F_viscosity " << F_viscosity << std::flush;
//    }
}

void Particle::computeOtherForces()
{
    Vector F_o = Vector();
    F_o = Vector(0, Settings::FORCES_GRAVITY_EARTH_VAL, 0) * m;
    *F += F_o;

//    if (Settings::FORCES_UNIVERSAL_GRAVITY)
//        for (unsigned i = 0; i < Particle::flows[parentFlow].size(); ++i)
//            Forces::universalGravitation(*this, *Particle::flows[parentFlow][i]);
}

void Particle::setModelMatrix()
{
    Matrices::modelMatrix.setToIdentity();
    Matrices::modelMatrix.translate(QVector3D(r->x, r->y, r->z));
    Matrices::modelMatrix.scale(QVector3D(radius, radius, radius));
    //Matrices::modelMatrix.rotate(100.0f * SimulationWindow::frame / SimulationWindow::refreshRate, 0, 1, 0);

    // x y z // r theta phi // radius inclination azimuth // yaw roll pitch
    Vector dir = v->normal();
    Vector up = Vector(0, 1, 0);
    double angle = - acos(dir.dot(up)) * radToDeg;
    if (fabs(angle) > 0.5)
    {
        Vector axis = (dir * up).normal();
        Matrices::modelMatrix.rotate(angle, QVector3D(axis.x, axis.y, axis.z)); // rotate towards velocity direction vector
    }
}

void Particle::paint()
{
    setModelMatrix();

    if (Settings::COLOR_BY_SPEED) {
        color[0] = (float) (v->norm() / v_max());
        color[1] = -1.;
        color[2] = -1.;
        Machine::recolor(color);
    }
    if (Settings::COLOR_BOUNDARIES) {
        if (boundary) {
            if (color[0] != 0.0f
             || color[1] != 1.0f
             || color[2] != 0.0f) {
                color[0] = 0.0f;
                color[1] = 1.0f;
                color[2] = 0.0f;
                Machine::recolor(color);
            }
        }
        else {
            if (color[0] != 0.5f
             || color[1] != 0.0f
             || color[2] != 0.0f) {
                color[0] = 0.5f;
                color[1] = 0.0f;
                color[2] = 0.0f;
                Machine::recolor(color);
            }
        }
    }
    Machine::paint();

    if (Settings::PAINT_VECTORS)
    {
        v->setApplicationPoint(*r);
        v->paint();
    }
}
