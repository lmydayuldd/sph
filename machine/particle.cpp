#include "machine/particle.h"

#include <iostream>
#include <random>

#include <omp.h>

#include "gl/matrices.h"
#include "machine/spring.h"
#include "physics/grid.h"
#include "physics/vector.h"
#include "shape/dot.h"
#include "shape/line.h"
#include "shape/sphere.h"
#include "util/constants.h"
#include "util/settings.h"

using namespace std;

std::vector<std::vector<Particle*>> Particle::flows;
int Particle::count = 0;

Particle::Particle()
{
}

Particle::~Particle()
{
    //delete parentFlow;
    delete r;
    delete v;
    delete a;
    delete dr;
    delete dv;
    delete da;
    delete F;
    for (unsigned int i = 0; i < springs.size(); ++i) {
        delete springs[i];
    }
}

Particle::Particle(int parentFlow)
    : parentFlow(parentFlow),
      id(++count),
      neighbours(nullptr),
      cell(std::vector<unsigned int>{ std::numeric_limits<unsigned int>::max(),
                                      std::numeric_limits<unsigned int>::max(),
                                      std::numeric_limits<unsigned int>::max() }),
      m(Settings::PARTICLE_MASS),
      rho(Settings::SPH_DESIRED_REST_DENSITY),
      charge(Settings::FORCES_COULOMB),
      temperature(Settings::SPH_TEMPERATURE),
      viscosity(Settings::SPH_VISCOSITY),
      pressure(Settings::SPH_PRESSURE),
      smoothing_length(Settings::SPH_SMOOTHING_LENGTH),
      radius(Settings::PARTICLE_RADIUS),
      stationary(false)
{
    r = new Vector(
        (-10 + radius) + fmod(id-1, 20) * radius,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
        (-10 + radius) + (id-1)/20 * radius,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
        0//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5)
    );
    v = new Vector(
        0,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%30),
        0,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%30),
        0//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%30)
    );
    a = new Vector();
    dr = new Vector();
    dv = new Vector();
    da = new Vector();
    F = new Vector();

    linkView(SPHERE);
}

void Particle::createView()
{
    float color[3] = {0.5, 0, 0};
    Sphere sphere(0.8, color);
    this->form = sphere.form;

//    Dot dot(color);
//    this->form = dot.form;
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
    for (unsigned int i = 0; i < Particle::flows[parentFlow].size(); ++i)
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
    // naive

//    unsigned int neighbour_count = Settings::SPH_NEIGHBOUR_COUNT;
//    double neighbour_range = std::numeric_limits<double>::infinity();//2 * smoothing_length;

//    unsigned int particle_count = Particle::flows[this->parentFlow].size();
//    if (neighbours == nullptr)
//    {
//        neighbours = new std::vector<Particle*>();
//    }
//    else
//    {
//        neighbours->clear();
//    }

//    double last_min_distance = 0;
//    for (unsigned int i = 0; i < neighbour_count && i < particle_count; ++i)
//    {
//        double min_distance = std::numeric_limits<double>::infinity();
//        unsigned int min_index = std::numeric_limits<unsigned int>::max();
//        for (unsigned int j = 0; j < particle_count; ++j)
//        {
//            if (this != Particle::flows[parentFlow][j])
//            {
//                double j_distance
//                        = r->distance(*Particle::flows[parentFlow][j]->r);
//                if (j_distance < min_distance && j_distance <= neighbour_range)
//                {
//                    if (j_distance > 0 && j_distance > last_min_distance)
//                    {
//                        min_distance = j_distance;
//                        min_index = j;
//                    }
//                }
//            }
//        }
//        if (min_index != std::numeric_limits<unsigned int>::max())
//        {
//            neighbours->push_back(Particle::flows[parentFlow][min_index]);
//        }
//        last_min_distance = min_distance;
//    }

    // mesh-supported

    unsigned int neighbour_count = Settings::SPH_NEIGHBOUR_COUNT;
    double neighbour_range = std::numeric_limits<double>::infinity();//2 * smoothing_length;

    unsigned int particle_count = Particle::flows[this->parentFlow].size();
    if (neighbours == nullptr)
    {
        neighbours = new std::vector<Particle*>();
    }
    else
    {
        neighbours->clear();
    }

    for (unsigned int i = std::max<unsigned int>(cell[0] - 1, 0);
         i <= std::min<unsigned int>(cell[0] + 1, Grid::cell_count); ++i)
    {
        for (unsigned int j = std::max<unsigned int>(cell[1] - 1, 0);
             j <= std::min<unsigned int>(cell[1] + 1, Grid::cell_count); ++j)
        {
            for (unsigned int k = std::max<unsigned int>(cell[2] - 1, 0);
                 k <= std::min<unsigned int>(cell[2] + 1, Grid::cell_count); ++k)
            {
                for (unsigned int p = 0; p < Grid::grid[i][j][k].size(); ++p)
                {
                    if (this != Grid::grid[i][j][k][p])
                    {
                        if (neighbours->size() < Settings::SPH_NEIGHBOUR_COUNT)
                        {
                            neighbours->push_back(Grid::grid[i][j][k][p]);
                        }
                    }
                }
            }
        }
    }

    // printout

    std::cout << id << ": ";
    for (unsigned int i = 0; i < neighbours->size(); ++i)
    {
        std::cout << (*neighbours)[i]->id << " ";
    }
    std::cout << std::endl;
}

/**
 * @brief Cubic spline smooth function
 */
double Particle::kernelFunction(Particle* p2)
{
    double h = smoothing_length;

    //double C_h = 1. / (6 * h);                  // for 1D
    double C_h = 15. / (14 * M_PI * pow(h, 2)); // for 2D
    //double C_h = 1. / (4 * M_PI * pow(h, 3));   // for 3D

    double kernel = 0;
    //std::cout << "distance " << r->distance(*p2->r) << std::endl << std::flush;
    double q = r->distance(*p2->r) / h;
    //std::cout << "QQQQQQQQQQ " << q << std::endl << std::flush;
    if      (q >= 0 && q < 1) kernel = C_h * (pow(2-q, 3) - 4 * pow(1-q, 3));
    else if (q >= 1 && q < 2) kernel = C_h *  pow(2-q, 3);
    else if (q >= 2)          kernel = 0;

    //std::cout << "Kernel " << kernel << std::endl << std::flush;
    return kernel;
}

Vector Particle::kernelFunctionGradient(Particle* p2)
{
    double h = smoothing_length;
    //double C_h = -1. / (6 * pow(h, 2)); // for 1D
    double C_h = -15. / (7 * M_PI * pow(h, 3)); // for 2D
    //double C_h = -3. / ((4 * M_PI) * pow(h, 4)); // for 3D

    double gradient_W = 0;
    double q = r->distance(*p2->r) / h;
//    if      (q >= 0 && q < 1) gradient_W = C_h * (-0.75 * (pow(q, 2) - 4*q + 4));
//    else if (q >= 1 && q < 2) gradient_W = C_h * (-pow(q, 3) + 2*pow(q, 2) - 4*q + 8);
//    else if (q >= 2)          gradient_W = 0;
    if      (q >= 0 && q < 1) gradient_W = C_h * q * (9*q - 4);
    else if (q >= 1 && q < 2) gradient_W = C_h * (q * (4 - 3*q) - 4);
    else if (q >= 2)          gradient_W = 0;

    //std::cout << "KernelFunction Gradient " << gradient_W_p1_p2 << std::endl << std::flush;
    return (*p2->r - *r) / r->distance(*p2->r) * gradient_W;
}

void Particle::updateDensity()
{
    if (neighbours->size() > 0)
    {
        rho = 0;
        for (unsigned int i = 0; i < neighbours->size(); ++i)
        {
            rho += (*neighbours)[i]->m * kernelFunction((*neighbours)[i]);
        }
    }
    //std::cout << "Density rho " << rho << std::endl << std::flush;
}

void Particle::updatePressure()
{
    double k = Settings::SPH_STIFFNESS_CONSTANT;
    double rho_0 = Settings::SPH_DESIRED_REST_DENSITY;

    pressure = k * (pow(rho/rho_0, 7) - 1);
    //std::cout << "Pressure " << pressure << std::endl << std::flush;
}

void Particle::computePressureForces()
{
    Vector gradient_pressure = Vector(); // approximated
    for (unsigned int i = 0; i < neighbours->size(); ++i)
    {
        Particle* n = (*neighbours)[i];
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
    for (unsigned int i = 0; i < neighbours->size(); ++i)
    {
        Particle* n = (*neighbours)[i];
        velocity_gradient_squared +=
            (*r - *n->r)
            * kernelFunctionGradient(n)
            * (v - n->v)
            * (n->m / n->rho)
            / ((*r - *n->r).dot(*r - *n->r)
               + 0.01 * pow(smoothing_length, 2));
    }
    velocity_gradient_squared *= 2;

    Vector F_v = Vector();
    F_v = velocity_gradient_squared * viscosity * m;
    *F += F_v;
    //std::cout << "F_viscosity " << F_v << std::endl << std::flush;
}

void Particle::computeOtherForces()
{
    Vector F_o = Vector();
    F_o = Vector(0, -9.81, 0) * m;
    *F += F_o;
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

    //float color[3] = {(float) (v->norm() / v_max()) , -1., -1.};
    //Machine::recolor(color);
    Machine::paint();

    if (Settings::PAINT_VECTORS)
    {
        v->setApplicationPoint(*r);
        v->paint();
    }
}

void Particle::collide(Particle* p2) {}
