#include "physics/forces.h"
#include "physics/vector.h"
#include "util/constants.h"

void Forces::universalGravitation(const Particle& p1, const Particle& p2)
{ // F1->2 = -G * m1 * m2 * r / |r|^2
    if (&p1 != &p2) {
        double M = p1.m * p2.m;
        Vector r21 = *p1.r - *p2.r;
        double d = r21.norm();
        r21 = r21.normal();
        if (! p1.stationary) *p1.F = *p1.F - r21 * (G_const * M/(d*d));
    }
}
void Forces::gravityEarth(const Particle& p)
{
    Vector gravity = Vector(0, - G_earth, 0);
    if (! p.stationary) *p.F = gravity * p.m;
}
void Forces::Coulomb(const Particle& p1, const Particle& p2)
{
    if (&p1 != &p2) {
        if (! p1.stationary) {
            double Q = p1.charge * p2.charge;
            Vector r21 = *p1.r - *p2.r;
            double d = r21.norm();
            r21 = r21.normal();
            *p1.F = *p1.F - r21 * (Coulomb_const * Q/(d*d));
        }
    }
}

void Forces::Friction(const Particle& p)
{
    if (! p.stationary) {
        Vector friction
                = Vector(*p.F).normal() * (-1 * coefficient_of_friction * p.F->y);

        *p.F = *p.F - friction;
    }
}

void Forces::Hooke(const Particle& p1, const Particle& p2,
                   double ks, double d, double kd)
{ // F-> = -ks . x-> // d = targetSpringDistance
    if (! p1.stationary || ! p2.stationary) {
        Vector r12 = *p1.r - *p2.r;
        Vector v12 = *p1.v - *p2.v;
        double fs = ks * fabs(r12.norm() - d);
        double fd = kd * v12.dotProduct(r12) / r12.norm();
        Vector fH = r12.normal() * (fs + fd);
        if (! p1.stationary) *p1.F = *p1.F - fH;
        if (! p2.stationary) *p2.F = *p2.F + fH;
    }
}

void Forces::collide(const Particle& p1, const Particle& p2) { // Elastic Collision
    // add damp(en)ing!!!

    double distanceFromCenter = p1.r->distance(*p2.r) - p2.radius;
    if (distanceFromCenter < p1.radius) {
        Vector sphereSurfaceNormal = (*p1.r - *p2.r).normal();
        double moveOutOfSphere = distanceFromCenter - p1.radius;// - p2.dr.norm();
        if (! p2.stationary)
            *p2.r = *p2.r + sphereSurfaceNormal * moveOutOfSphere;

        Vector p1_v_new = *p1.v - (
            (*p1.r - *p2.r) * (
                (*p1.v - *p2.v)
                    .dotProduct(*p1.r - *p2.r)
                    / pow(p1.r->distance(*p2.r), 2)
                    * 2 * p2.m / (p1.m + p2.m)
            )
        );
        if (! p2.stationary)
            *p2.v = *p2.v - (
                (*p2.r - *p1.r) * (
                    (*p2.v - *p1.v)
                        .dotProduct(*p2.r - *p1.r)
                        / pow(p2.r->distance(*p1.r), 2)
                        * 2 * p1.m / (p1.m + p2.m)
                )
            );
        if (! p1.stationary) {
            *p1.v = p1_v_new;
        }
    }
}

//void CollisionSphere::collide_new(Particle& p2) {
//    double nextDistance = position.distance(p2.r + p2.v) - p2.radius; // d = |p1-p2|-r2
//    if (nextDistance < radius) { // collision
//        Vector sphereSurfaceNormal = (position - p2.r).normal();      // ssn = p1-p2/|p1-p2|
//        double moveOutOfSphere = nextDistance - radius;               // move = d-r1
//        p2.r = p2.r + sphereSurfaceNormal * moveOutOfSphere;          // p2 = p2 + ssn*move

//        Vector p2_v_n = sphereSurfaceNormal * sphereSurfaceNormal.dotProduct(p2.v);
//        Vector p2_v_s = p2.v - p2_v_n;
//        p2.v = p2_v_s - p2_v_n * damping;
//    }
//}

//void CollisionSphere::collide(Particle& p2) {
//    double distance = position.distance(p2.r) - p2.radius;       // d = |p1-p2|-r2
//    if (distance < radius) { // collision
//        Vector sphereSurfaceNormal = (position - p2.r).normal(); // ssn = p1-p2/|p1-p2|
//        double moveOutOfSphere = distance - radius;              // move = d-r1
//        p2.r = p2.r + sphereSurfaceNormal * moveOutOfSphere;     // p2 = p2 + ssn*move

//        Vector p2_v_n = sphereSurfaceNormal * sphereSurfaceNormal.dotProduct(p2.v);
//        Vector p2_v_s = p2.v - p2_v_n;
//        p2.v = p2_v_s - p2_v_n * damping;
//    }
//}
