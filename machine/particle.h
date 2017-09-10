#ifndef PARTICLE_H
#define PARTICLE_H

class Spring;
class Vector;
class Octree;

#include "machine/machine.h"

class Particle : public Machine
{
public:
    static int count;
    static std::vector<std::vector<Particle*>> flows;
    static std::vector<std::vector<bool>> collision;
    static std::vector<std::vector<double>> collisionDistance;
    //std::vector<Particle*>* parentFlow;
    static double v_max_norm, F_P_max_norm, F_visc_max_norm, F_tens_max_norm;
    static double v_avg_norm, F_P_avg_norm, F_visc_avg_norm, F_tens_avg_norm;
// for CUDA
    static double *rx_host, *ry_host, *rz_host;
    static double *vx_host, *vy_host, *vz_host;
    static double *m_host;
    static bool *is_stationary_host;
    static double *rx_device, *ry_device, *rz_device;
    static double *vx_device, *vy_device, *vz_device;
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
    Vector *F_P, *F_visc, *F_tens;
    double m, rho, charge, viscosity, P;
    double radius;
    double smoothing_length;
    double colorTag;
    bool isStationary, isGhost, isPhantom;
    bool boundary, overpressured;
    bool *didCollide;
    double *dt_left;
    float colorDefault[3];
    float colorBoundary[3];
    float colorOverpressured[3];
    float colorGreen[3];
    Form *formDefault = nullptr;
    Form *formBoundary = nullptr;
    Form *formOverpressured = nullptr;
    Form *formGreen = nullptr;

    Particle();
    Particle(int parentFlow);
    ~Particle();

    static void evaluateMaximalVelocity();
    static void evaluateMaximalPressure();
    static void evaluateMaximalViscosity();
    static void evaluateMaximalTension();

    void updateNeighbours();
    void isBoundary();
    void updateDensity();
    void updatePressure();
    void computePressureForces();
    void computeViscosityForces();
    void computeSurfaceTensionForces();
    void computeOtherForces();
    double kernelFunction(Particle *p2);
    Vector kernelFunctionGradient(Particle *p2);
    Vector kernelFunctionGradientSquared(Particle *p2);
    void paintLayers();
    void paintBoundaries();
    void paintDensities();
    void paintVelocities();
    void paintPressures();
    void paintViscosities();
    void paintTensions();
    void springify(Particle *p2, float ks, float d, float kd);
    void springifyMutual(Particle *p2, float ks, float d, float kd);

    virtual void createView() override;
    virtual void setModelMatrix() override;
    virtual void paint() override;
};

#endif // PARTICLE_H
