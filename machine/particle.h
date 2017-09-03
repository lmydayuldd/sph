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
    static std::vector<std::vector<bool>> collision;
    static std::vector<std::vector<double>> collisionDistance;
    //std::vector<Particle*>* parentFlow;
// for CUDA
    static double *rx_host;
    static double *ry_host;
    static double *rz_host;
    static double *vx_host;
    static double *vy_host;
    static double *vz_host;
    static double *m_host;
    static bool *is_stationary_host;
    static double *rx_device;
    static double *ry_device;
    static double *rz_device;
    static double *vx_device;
    static double *vy_device;
    static double *vz_device;
    static double *m_device;
    static bool *is_stationary_device;
// for CUDA
    int parentFlow;
    int id;
    std::vector<Particle*> *neighbours;
    std::vector<unsigned> cell;
    Octree *cube;
    std::vector<Spring*> springs;
    Vector *r, *v, *a, *dr, *dv, *da, *F;
    Vector *r_former, *v_former;
    double m, rho, charge, temperature, viscosity, P;
    double smoothing_length;
    double radius;
    bool isStationary;
    bool *didCollide;
    bool boundary;
    bool overpressured;
    float colorDefault[3];
    float colorBoundary[3];
    float colorOverpressured[3];
    double *dt_left;
    Form *formDefault = nullptr;
    Form *formBoundary = nullptr;
    Form *formOverpressured = nullptr;
    double color = 0.;

    Particle();
    Particle(int parentFlow);
    ~Particle();

    void springify(Particle *p2, float ks, float d, float kd);
    void springifyMutual(Particle *p2, float ks, float d, float kd);
    double v_max();
    void updateNeighbours();
    void isBoundary();
    double kernelFunction(Particle *p2);
    Vector kernelFunctionGradient(Particle *p2);
    Vector kernelFunctionGradientSquared(Particle *p2);
    void updateDensity();
    void updatePressure();
    void computePressureForces();
    void computeViscosityForces();
    void computeSurfaceTensionForces();
    void computeOtherForces();

    virtual void createView() override;
    virtual void setModelMatrix() override;
    virtual void paint() override;
};

#endif // PARTICLE_H
