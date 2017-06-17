#ifndef PARTICLE_H
#define PARTICLE_H

class Spring;
class Vector;
class Octree;

#include "machine/machine.h"

class Particle : public Machine
{
public:
    static std::vector<std::vector<Particle*>> flows;
    static int count;
    //std::vector<Particle*>* parentFlow;
    int parentFlow;
    int id;
    std::vector<Particle*>* neighbours;
    std::vector<unsigned int> cell;
    Octree* cube;
    std::vector<Spring*> springs;
    Vector *r, *v, *a, *dr, *dv, *da, *F;
    Vector *r_former, *v_former;
    double m, rho, charge, temperature, viscosity, pressure;
    double smoothing_length;
    double radius;
    bool stationary;
    bool *didCollide;
    bool boundary;
    float color[3];

    Particle();
    Particle(int parentFlow);
    ~Particle();

    void springify(Particle* p2, float ks, float d, float kd);
    void springifyMutual(Particle* p2, float ks, float d, float kd);
    double v_max();
    void updateNeighbours();
    void isBoundary();
    double kernelFunction(Particle* p2);
    Vector kernelFunctionGradient(Particle* p2);
    void updateDensity();
    void updatePressure();
    void computePressureForces();
    void computeViscosityForces();
    void computeOtherForces();

    virtual void createView() override;
    virtual void setModelMatrix() override;
    virtual void paint() override;
    virtual void collide(Particle* p2) override;
};

#endif // PARTICLE_H
