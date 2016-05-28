#ifndef PARTICLE_H
#define PARTICLE_H

class Flow;
class Spring;
class Vector;

#include "machine/machine.h"

class Particle : public Machine
{
public:
    static std::vector<std::vector<Particle*>> flows;
    std::vector<Particle*>* parentFlow;
    std::vector<Spring*> springs;
    Vector *r, *v, *a, *dr, *dv, *da, *F;
    double m, rho, charge, temperature, viscosity;
    double kernel;
    double radius = 1.;
    bool stationary;

    Particle();
    Particle(std::vector<Particle*>* parentFlow);
    ~Particle();

    void createView() override;
    void setModelMatrix() override;
    void paint() override;
    void collide(Particle *p2) override;

    void springify(Particle* p2, float ks, float d, float kd);
    void springifyMutual(Particle* p2, float ks, float d, float kd);
};

#endif // PARTICLE_H
