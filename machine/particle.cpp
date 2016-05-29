#include "machine/particle.h"

#include <iostream>
#include <random>

#include "gl/matrices.h"
#include "machine/spring.h"
#include "physics/vector.h"
#include "shape/dot.h"
#include "shape/sphere.h"
#include "util/constants.h"
#include "util/settings.h"

using namespace std;
#include "shape/line.h"

std::vector<std::vector<Particle*>> Particle::flows;

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
      m(1.0),
      rho(1.0),
      charge(1.0),
      temperature(1.0),
      viscosity(1.0),
      kernel(1.0),
      radius(0.1),
      stationary(false)
{
    r = new Vector(
        (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
        (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
        0//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5)
    );
    v = new Vector();
    if (Settings::PARTICLES_INITIAL_SPEED) {
        v = new Vector(
            (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%30),
            (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%30),
            0//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%30)
        );
    }
    a = new Vector();
    dr = new Vector();
    dv = new Vector();
    da = new Vector();
    F = new Vector();

    linkView(SPHERE);
}

void Particle::createView()
{
//    float position[3] = {0, 0, 0};
//    Dot dot(position, color);
//    this->forms.push_back(dot.form);

    float color[3] = {1, 0, 0};
    Sphere sphere(0.8, color);
    this->form = sphere.form;
}

void Particle::springify(Particle* p2, float ks, float d, float kd)
{
    springs.push_back(new Spring(p2, ks, d, kd));
}
void Particle::springifyMutual(Particle* p2, float ks, float d, float kd)
{
    springs.push_back(new Spring(p2, ks, d, kd));
    p2->springs.push_back(new Spring(this, ks, d, kd)); ////////////////////////////////
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
    double angle = - acos(dir.dotProduct(up)) * radToDeg;
    if (fabs(angle) > 0.5) {
        Vector axis = (dir * up).normal();
        Matrices::modelMatrix.rotate(angle, QVector3D(axis.x, axis.y, axis.z)); // rotate towards velocity direction vector
    }
}

void Particle::paint()
{
    setModelMatrix();

    Machine::paint();

    v->setApplicationPoint(*r);
    v->paint();
}

void Particle::collide(Particle* p2) {}
