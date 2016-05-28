#ifndef FORCES_H
#define FORCES_H

#include <cmath>

#include "machine/particle.h"

class Forces {
private:
    static constexpr double G_const = 6.673 * pow(10, -11);
    static constexpr double sigma_0 = 8.8541878176 * pow(10, -12); // electric permittivity of free space (vacuum) [F/m] [farads/m]
    static constexpr double Coulomb_const = 8.987 * pow(10, 9); //1 / (4 * PI * sigma_0); // Coulomb force constant [ N * m^2/C^2 ]
    //static constexpr double coefficient_of_friction = Settings.FORCES_FRICTION;
    static constexpr double coefficient_of_friction = 0.5;
    static constexpr double G_earth = 9.81;

public:
    static void universalGravitation(
        const Particle& p1, const Particle& p2
    ); // F1->2 = -G * m1 * m2 * r / |r|^2
    static void gravityEarth(const Particle& p);
    static void Coulomb(const Particle& p1, const Particle& p2);
    static void Friction(const Particle& p);
    static void Hooke(
        const Particle& p1, const Particle& p2,
        double ks, double d, double kd
    ); // F-> = -ks . x-> // d = targetSpringDistance
    static void collide(const Particle& p1, const Particle& p2); // Elastic Collision
};

#endif // FORCES_H
