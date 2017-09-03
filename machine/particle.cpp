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

#ifdef COMPILER_GPP
    #define LOOP_TYPE unsigned
#elif COMPILER_MSVC
    #define LOOP_TYPE int
#endif

using namespace std;

std::vector<std::vector<Particle*>> Particle::flows;
int Particle::count = 0;
std::vector<std::vector<bool>> Particle::collision;
std::vector<std::vector<double>> Particle::collisionDistance;
// for CUDA
// pinned host memory
double *Particle::rx_host = 0;
double *Particle::ry_host = 0;
double *Particle::rz_host = 0;
double *Particle::vx_host = 0;
double *Particle::vy_host = 0;
double *Particle::vz_host = 0;
double *Particle::m_host = 0;
bool *Particle::is_stationary_host = 0;
double *Particle::rx_device = 0;
double *Particle::ry_device = 0;
double *Particle::rz_device = 0;
double *Particle::vx_device = 0;
double *Particle::vy_device = 0;
double *Particle::vz_device = 0;
double *Particle::m_device = 0;
bool *Particle::is_stationary_device = 0;
// for CUDA

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
      P(Settings::PRESSURE),
      smoothing_length(Settings::SMOOTHING_LENGTH),
      radius(Settings::PARTICLE_RADIUS),
      isStationary(false),
      didCollide(new bool(false)),
      boundary(false),
      overpressured(false),
      colorDefault {0.5, 0, 0},
      colorBoundary {0.2f, 0.2f, 0.2f},
      colorOverpressured {0.7f, 0, 0},
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
                (-Settings::ARENA_DIAMETER/2 + Settings::PARTICLES_INIT_DIST/2) + fmod(id-1, 20) * Settings::PARTICLES_INIT_DIST,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
                (-Settings::ARENA_DIAMETER/2 + Settings::PARTICLES_INIT_DIST/2) + (id-1)/20 * Settings::PARTICLES_INIT_DIST,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
                0//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5)
            );
        break;
        case RANDOM_NON_MAP :
            r = new Vector(
                (-Settings::ARENA_DIAMETER/2 + Settings::PARTICLES_INIT_DIST/2) + (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * Settings::ARENA_DIAMETER,
                (-Settings::ARENA_DIAMETER/2 + Settings::PARTICLES_INIT_DIST/2) + (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * Settings::ARENA_DIAMETER,
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
    formDefault = currentForm;
    linkView(SPHERE_BOUNDARY);
    formBoundary = currentForm;
    linkView(SPHERE_OVERPRESSURED);
    formOverpressured = currentForm;
}

void Particle::createView()
{
    if (Settings::DOT_OR_SPHERE) {
        Sphere sphereDefault(1.0, colorDefault, SPHERE);
        formDefault = sphereDefault.form;
        Sphere sphereBoundary(1.0, colorBoundary, SPHERE_BOUNDARY);
        formBoundary = sphereBoundary.form;
        Sphere sphereOverpressured(1.0, colorOverpressured, SPHERE_OVERPRESSURED);
        formOverpressured = sphereOverpressured.form;
        if      (boundary)      currentForm = formBoundary;
        else if (overpressured) currentForm = formOverpressured;
        else                    currentForm = formDefault;
    }
    else
    {
        Dot dot(colorDefault);
        this->currentForm = dot.form;
    }
}

void Particle::springify(Particle *p2, float ks, float d, float kd)
{
    springs.push_back(new Spring(p2, ks, d, kd));
}
void Particle::springifyMutual(Particle *p2, float ks, float d, float kd)
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
        for (int i = std::max<int>((int)cell[0] - 1, 0);
             i <= std::min<unsigned>(cell[0] + 1, Grid::cell_count - 1);
             ++i) {
            for (int j = std::max<int>((int)cell[1] - 1, 0);
                 j <= std::min<unsigned>(cell[1] + 1, Grid::cell_count - 1);
                 ++j) {
                for (int k = std::max<int>((int)cell[2] - 1, 0);
                     k <= std::min<unsigned>(cell[2] + 1, Grid::cell_count - 1);
                     ++k) {
                    for (int l = 0;
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
                      << " <- Particle 50 neighbour count." << std::endl << std::flush;
        }
        if (id == 20) {
            std::cout << neighbours->size()
                      << " <- Particle 20 neighbour count." << std::endl << std::flush;
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
//    if (neighbours->size() <= 0)
//    {
//        boundary = true;
//    }
//    else {
//        boundary = false;
//    }\

    if (abs(rho - Settings::DESIRED_REST_DENSITY)
        > Settings::DESIRED_REST_DENSITY * 0.4) // TODO, 0.7? that's a lot. Why differences in density are so big?
    {
        if (rho < Settings::DESIRED_REST_DENSITY) {
            boundary = true;
            overpressured = false;
            color = 0;
        }
        else {
            overpressured = true;
            boundary = false;
            color = 1;
        }
    }
    else {
        boundary = false;
        overpressured = false;
        color = 1;
    }
}

/**
 * @brief Cubic spline smooth function
 */
double Particle::kernelFunction(Particle *p2)
{
    double h = smoothing_length;
    double C_h = 0.;
    double W = 0.;
    double q = r->distance(*p2->r) / h;

    switch (Settings::KERNEL_DIM)
    {
        // Cubic Spline
        case 1: C_h = 1.  / ( 6        * h);         break;
        case 2: C_h = 15. / (14 * M_PI * pow(h, 2)); break;
        case 3: C_h = 1.  / ( 4 * M_PI * pow(h, 3)); break;
//        // Quintic Spline SPH kernel: [Liu2010]
//        case 1: C_h = 1. / (120 * h);                break;
//        case 2: C_h = 7. / (478 * M_PI * pow(h, 2)); break;
//        case 3: C_h = 3. / (359 * M_PI * pow(h, 3)); break;
    }

    // Cubic Spline
    if      (q >= 0. && q < 1.) W = C_h * (pow(2-q, 3) - 4 * pow(1-q, 3));
    else if (q >= 1. && q < 2.) W = C_h *  pow(2-q, 3);
    else if (q >= 2.)           W = 0.;
//    // Quintic Spline SPH kernel: [Liu2010]
//    if      (q >= 0. && q <= 1.) W = C_h * (pow(3-q, 5) - 6*pow(2-q, 5) + 15*pow(1-q, 5));
//    else if (q >  1. && q <= 2.) W = C_h * (pow(3-q, 5) - 6*pow(2-q, 5));
//    else if (q >  2. && q <= 3.) W = C_h *  pow(3-q, 5);
//    else if (q >  3.)            W = 0;

    //std::cout << "distance " << r->distance(*p2->r) << std::endl << std::flush;
    //std::cout << "QQQQQQQQQQ " << q << std::endl << std::flush;
    //std::cout << "Kernel " << W << std::endl << std::flush;
    return W;
}

Vector Particle::kernelFunctionGradient(Particle *p2)
{
    double h = smoothing_length;
    double C_h = 0.;
    double grad_W = 0;
    double q = r->distance(*p2->r) / h;

    switch (Settings::KERNEL_DIM)
    {
        // Cubic Spline
        case 1: C_h =  -1. / (6        * pow(h, 2)); break;
        case 2: C_h = -15. / (7 * M_PI * pow(h, 3)); break;
        case 3: C_h =  -3. / (4 * M_PI * pow(h, 4)); break;
//        // Quintic Spline SPH kernel: [Liu2010]
//        case 1: C_h = -1. / (120        * pow(h, 2)); break;
//        case 2: C_h = -7. / (239 * M_PI * pow(h, 3)); break;
//        case 3: C_h = -9. / (359 * M_PI * pow(h, 4)); break;
    }

    // Cubic Spline
    if      (q >= 0. && q < 1.) grad_W = C_h *  q * (9*q - 4);
    else if (q >= 1. && q < 2.) grad_W = C_h * (q * (4 - 3*q) - 4);
    else if (q >= 2.)           grad_W = 0;
//    // Quintic Spline SPH kernel: [Liu2010]
//    if      (q >= 0. && q <= 1.) grad_W = C_h * (-10 * q * (5*pow(q, 3) - 12*pow(q, 2) + 12));
//    else if (q >  1. && q <= 2.) grad_W = C_h * (30*pow(2-q, 4) - 5*pow(3-q, 4));
//    else if (q >  2. && q <= 3.) grad_W = C_h * (-5 * pow(3-q, 4));
//    else if (q >  3.)            grad_W = 0;

    //std::cout << "Kernel Function gradient " << grad_W << std::endl << std::flush;
    return (*p2->r - *r) / r->distance(*p2->r) * grad_W;
}

Vector Particle::kernelFunctionGradientSquared(Particle *p2)
{
//    double h = smoothing_length;
//    double C_h = 0.;
//    double grad_W = 0;
//    double q = r->distance(*p2->r) / h;

//    switch (Settings::KERNEL_DIM)
//    {
//        case 1: C_h =  -1. / (6        * pow(h, 2)); break;
//        case 2: C_h = -15. / (7 * M_PI * pow(h, 3)); break;
//        case 3: C_h =  -3. / (4 * M_PI * pow(h, 4)); break;
//    }

//    if      (q >= 0. && q < 1.) grad_W = C_h *  q * (9*q - 4);
//    else if (q >= 1. && q < 2.) grad_W = C_h * (q * (4 - 3*q) - 4);
//    else if (q >= 2.)           grad_W = 0;

//    //std::cout << "Kernel Function gradient " << grad_W << std::endl << std::flush;
//    return (*p2->r - *r) / r->distance(*p2->r) * grad_W * grad_W;
    return Vector();
}

void Particle::updateDensity()
{
    ///////////// TODO, why save density if no neighbours?
    if (neighbours->size() > 0)
    {
        rho = 0.;
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
    double gamma = 7; // for water

    P = k * (pow(rho/rho_0, gamma) - 1); // Tait equation

//    double c_s = Settings::SOUND_SPEED;
//    pressure = c_s * c_s * (rho - rho_0);

    //std::cout << "Pressure " << pressure << std::endl << std::flush;
}

void Particle::computePressureForces()
{
    // TODO // what to do when no neighbours
    if (neighbours->size() > 0)
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
//            gradient_pressure +=
//                kernelFunctionGradient(n)
//                * n->m
//                * (P/pow(rho, 2) + n->P/pow(n->rho, 2));
            gradient_pressure +=
                kernelFunctionGradient(n)
                * n->m
                * (P + n->P)
                / (2 * rho * n->rho);
            // TODO // chose one of methods above
        }
        gradient_pressure *= rho;

        Vector F_p = Vector();
        F_p = gradient_pressure * (- m/rho);
        *F += F_p;
        //std::cout << "F_pressure " << F_p << std::endl << std::flush;
    }
}

void Particle::computeViscosityForces()
{
    // TODO // what to do when no neighbours
    if (neighbours->size() > 0)
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
        F_viscosity = velocity_gradient_squared * viscosity * m;// * 10000000000000000000;
        *F += F_viscosity;
//#pragma omp critical
        if (id == 500) {
          std::cout << "p500_F_viscosity " << F_viscosity << std::endl << std::flush;
        }
    }
}

void Particle::computeSurfaceTensionForces()
{
    if (neighbours->size() > 0)
    {
        double color_avg = 0.; // 1 or 0 only
        Vector color_gradient = Vector(); // approximated
        Vector color_gradient_squared = Vector();

        Particle *n = nullptr;
        for (unsigned i = 0; i < neighbours->size(); ++i)
        {
            n = (*neighbours)[i];
            color_avg += kernelFunction(n) * n->color * n->m / n->rho;
            color_gradient += kernelFunctionGradient(n) * n->color * n->m / n->rho;
            color_gradient_squared
                += kernelFunctionGradientSquared(n) * n->color * n->m / n->rho;
        }

        double surfaceTensionCoefficient = Settings::SURFACE_TENSION_COEF;
        Vector liquid_surface_normal = Vector();
        Vector F_st = Vector();
        if (color_gradient.norm() != 0.) {
            liquid_surface_normal = - color_gradient / color_gradient.norm();

//            double surface_curvature = 0.;
//            //double liquid_surface_normal_gradient = 0.;
//            surface_curvature = kernelFunctionGradient
//            F_st = liquid_surface_normal * surfaceCurvature * surfaceTensionCoefficient;

            F_st = liquid_surface_normal * surfaceTensionCoefficient;
            *F += F_st;
        }

//#pragma omp critical
        if (id == 500) {
            //        std::cout << color << " color" << std::endl << std::flush;
            //        std::cout << color_avg << " color_avg" << std::endl << std::flush;
            std::cout << F_st << " p500_tension" << std::endl << std::flush;
        }



//        double gamma = 3.;

//        Vector F_cohesion = Vector();
//        Particle *n = nullptr;
//        for (unsigned i = 0; i < neighbours->size(); ++i)
//        {
//            n = (*neighbours)[i];
//            double h = Settings::SMOOTHING_LENGTH;
//            double d = r->distance(*n->r);
//            double C = 32. / (M_PI * pow(h, 9));
//            if      (2*r > h && r <= h) C *=     pow(h-r, 3) * pow(r, 3);
//            else if (r > 0 && 2*r <= h) C *= 2 * pow(h-r, 3) * pow(r, 3) - pow(h, 6)/64;
//            else                        C  = 0;
//            F_cohesion += (*r - *n->r) * (- gamma * m * n->m * C);
//        }

//        Vector liquid_surface_normal = Vector();
//        for (unsigned i = 0; i < neighbours->size(); ++i)
//        {
//            n = (*neighbours)[i];
//            liquid_surface_normal += n->m / n->rho * kernelFunctionGradient(n);
//        }
//        liquid_surface_normal *= h;

//        Vector F_curvature = Vector();
    }
}

void Particle::computeOtherForces()
{
    Vector F_o = Vector();
    F_o = Vector(0, Settings::FORCES_GRAVITY_EARTH_VAL, 0) * m;
    *F += F_o;

    if (Settings::FORCES_GRAVITY_UNIVERSAL)
        for (unsigned i = 0; i < Particle::flows[parentFlow].size(); ++i)
            Forces::universalGravitation(*this, *Particle::flows[parentFlow][i]);
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

//    if (Settings::COLOR_BY_SPEED) {
//        color[0] = (float) (v->norm() / v_max());
//        color[1] = -1.;
//        color[2] = -1.;
//        Machine::recolor(color);
//    }
//    if (Settings::COLOR_BOUNDARIES) {
//        if (boundary) {
//            if (color[0] != 0.0f
//             || color[1] != 1.0f
//             || color[2] != 0.0f) {
//                color[0] = 0.0f;
//                color[1] = 1.0f;
//                color[2] = 0.0f;
//                Machine::recolor(color);
//            }
//        }
//        else {
//            if (color[0] != 0.5f
//             || color[1] != 0.0f
//             || color[2] != 0.0f) {
//                color[0] = 0.5f;
//                color[1] = 0.0f;
//                color[2] = 0.0f;
//                Machine::recolor(color);
//            }
//        }
//    }

    if      (boundary) {
        linkView(SPHERE_BOUNDARY);
    }
    else if (overpressured) {
        linkView(SPHERE_OVERPRESSURED);
    }
    else {
        linkView(SPHERE);
    }
//    if (collision[499][id-1] || collision[id-1][499]) {
//        linkView(SPHERE_BOUNDARY);
//    }
//    else if (id == 500) {
//        linkView(SPHERE_OVERPRESSURED);
//    }
//    else {
//        linkView(SPHERE);
//    }
    Machine::paint();

    if (Settings::PAINT_VECTORS)
    {
        v->setApplicationPoint(*r);
        v->paint();
    }
}
