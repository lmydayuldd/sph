#ifndef FORCES_H
#define FORCES_H

#include <cmath>

#include "machine/particle.h"

class Forces {
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
    // Elastic Collision
    static bool doCollide(const Particle& p1, const Particle& p2);
    static void collisionDetect();
    static void collide(const Particle& p1, const Particle& p2);
    static void collide(const Particle& p1, std::vector<Particle*>*);
};

#endif // FORCES_H
