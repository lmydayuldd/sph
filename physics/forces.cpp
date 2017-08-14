#include "physics/forces.h"

#include "physics/vector.h"
#include "util/constants.h"
#include "util/operations.h"
#include "util/settings.h"

//#include <cstring> // for std::memset
#include <limits>

void Forces::universalGravitation(const Particle &p1, const Particle &p2)
{ // F1->2 = -G * m1 * m2 * r / |r|^2
    if (&p1 != &p2) {
        double M = p1.m * p2.m;
        Vector r21 = *p1.r - *p2.r;
        double d = r21.norm();
        r21 = r21.normal();
        if (! p1.stationary) *p1.F = *p1.F - r21 * (G_const * M/(d*d));
    }
}

void Forces::gravityEarth(const Particle &p)
{
    Vector gravity = Vector(0, - G_earth, 0);
    if (! p.stationary) *p.F += gravity * p.m;
}

void Forces::Coulomb(const Particle &p1, const Particle &p2)
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

void Forces::Friction(const Particle &p)
{
    if (! p.stationary) {
        Vector friction
            = Vector(*p.F).normal() * (-1 * coefficient_of_friction * p.F->y);

        *p.F = *p.F - friction;
    }
}

void Forces::Hooke(const Particle &p1, const Particle &p2,
                   double ks, double d, double kd)
{ // F-> = -ks . x-> // d = targetSpringDistance
    if (! p1.stationary || ! p2.stationary) {
        Vector r12 = *p1.r - *p2.r;
        Vector v12 = *p1.v - *p2.v;
        double fs = ks * fabs(r12.norm() - d);
        double fd = kd * v12.dot(r12) / r12.norm();
        Vector fH = r12.normal() * (fs + fd);
        if (! p1.stationary) *p1.F = *p1.F - fH;
        if (! p2.stationary) *p2.F = *p2.F + fH;
    }
}

bool Forces::doCollide(const Particle &p1, const Particle &p2) { // Elastic Collision
    bool doCollide = false;
    double distanceBorder = p1.r->distance(*p2.r) - p1.radius - p2.radius;
    if (distanceBorder < 0.) {
        doCollide = true;
#pragma omp critical
        {
            *p1.didCollide = true;
            *p2.didCollide = true;
            Particle::collision[p1.id - 1][p2.id - 1] = true;
            Particle::collisionDistance[p1.id - 1][p2.id - 1] = distanceBorder;
        }
    }
    return doCollide;
}

void Forces::collisionDetect()
{
    for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
        for (unsigned j = 0; j < Particle::flows[0].size(); ++j)
            doCollide(*Particle::flows[0][i], *Particle::flows[0][j]);
}

void Forces::collide(const Particle &p1, const Particle &p2) {
    //if (! *p1.didCollide)
    {
        if (doCollide(p1, p2)) {
            double exactCollisionTime = 0.;
            if (Settings::COLLIDE_MOVE_OUT)
            {
                Vector sphereNormal = (*p1.r - *p2.r).normal();
                double distanceBorder
                    = p1.r->distance(*p2.r) - p1.radius - p2.radius;

                if (! p2.stationary)
                {
#pragma omp critical
                    *p2.r += sphereNormal * std::min(distanceBorder, Settings::PARTICLE_MAX_DR);
                }

//                Vector dist = *p1.r_former - *p2.r_former;
//                Vector relative_v = *p1.v - *p2.v;
//                double radiuses = p1.radius + p2.radius;
//                double a = relative_v.dot(relative_v);
//                double b = relative_v.dot(dist);
//                double c = dist.dot(dist) - radiuses * radiuses;
//                double d = b*b - a*c;
//                if (c < 0.) { // already collided
//                    exactCollisionTime  = 0.;
//                }
//                else if (b >= 0.) { // don't move towards each other

//                }
//                else if (d < 0.) { // no roots, no collision

//                }
//                else if (b < 0. && d >= 0.) {
//                    exactCollisionTime  = (-b - sqrt(d)) / a;
//                    std::cout << "t: " << exactCollisionTime  << std::endl << std::flush;
//                    *p1.r = *p1.r_former + *p1.v * exactCollisionTime;
//                    *p2.r = *p2.r_former + *p2.v * exactCollisionTime;
//                    std::cout << "distance: " << p1.r->distance(*p2.r) << std::endl << std::flush;
//                }
            }

            if (Settings::COLLIDE_TRANSFER_VEL)
            {
                Vector p2_v(*p2.v);
                if (! p2.stationary)
                {
#pragma omp critical
                    *p2.v -=
                        (*p2.r - *p1.r) * (
                            (*p2.v - *p1.v)
                                .dot(*p2.r - *p1.r)
                                / pow(p2.r->distance(*p1.r), 2)
                                * 2 * p1.m / (p1.m + p2.m)
                        );
//#pragma omp critical
//                    *p2.v *= (1. - Settings::WATER_DAMPENING);
                }
                if (! p1.stationary)
                {
#pragma omp critical
                    *p1.v -=
                        (*p1.r - *p2.r) * (
                            (*p1.v - p2_v)
                                .dot(*p1.r - *p2.r)
                                / pow(p1.r->distance(*p2.r), 2)
                                * 2 * p2.m / (p1.m + p2.m)
                        );
                    //*p1.v *= (1. - Settings::WATER_DAMPENING);
                }
                if (! p1.stationary) {
//                    *p1.r += *p1.v * (Settings::dt - exactCollisionTime);//(1. - exactCollisionTime); //// ??
//                    *p1.r_former = *p1.r;
                }
                if (! p2.stationary) {
//                    *p2.r += *p2.v * (Settings::dt - exactCollisionTime);//(1. - exactCollisionTime); //// ??
//                    *p2.r_former = *p2.r;
                }
            }
        }
    }
}

void Forces::collide(const Particle &p1, std::vector<Particle*>* with) {
    //std::vector<Particle*> *neighbours = p1.neighbours;
    //with = neighbours; /////////////////////////////////
    //std::vector<Particle*> *parentFlow = &Particle::flows[p1.parentFlow];
    //std::vector<std::vector<Particle*>> *flows = &Particle::flows;

    double former_p1_dt_left = *p1.dt_left;
    Particle *p2 = nullptr;
    while (*p1.dt_left > 0. && fabs(former_p1_dt_left - *p1.dt_left) > 0.01)
    {
        double formerECT = 0.; // former exact collision time
        for (unsigned i = 0; i < with->size(); ++i)
        {
            p2 = with->at(i);
            if (p2->id != p1.id)
            {
                double eCT = 0.; // exact collision time
                Vector dist = *p1.r_former - *p2->r_former;
                Vector relative_v = *p1.v - *p2->v;
                double radiuses = p1.radius + p2->radius;
                double a = relative_v.dot(relative_v);
                double b = relative_v.dot(dist);
                double c = dist.dot(dist) - radiuses * radiuses;
                double d = b*b - a*c;

                if (c < 0.) { // already overlap
                    eCT = 0.;
                }
                if (b >= 0.) { // don't move towards each other
                    continue; /////////////////////////////////////////
                }
                if (d < 0.) { // no roots, no collision
                    continue; /////////////////////////////////////////
                }
                if (b < 0. && d >= 0.) {
                    eCT = (-b - sqrt(d)) / a;
                }

                if (fabs(eCT - formerECT) < 0.01) { // paricle is most likely locked between others
                    continue; ////////////////////////////////////////////////////
                }
                if (eCT < 0.01 || eCT > *p1.dt_left || eCT > *p2->dt_left) {
                    continue; ////////////////////////////////////////////////////
                }
                formerECT = eCT;

                *p1.didCollide = true;
#pragma omp critical
                *p2->didCollide = true;
                Particle::collision[p1.id - 1][p2->id - 1] = true;

                *p1.dt_left -= eCT;
                *p2->dt_left -= eCT;
                *p1.r = *p1.r_former + *p1.v * eCT;
                *p2->r = *p2->r_former + *p2->v * eCT;

                std::cout << "distance - r's = "
                          << p1.r->distance(*p2->r) - p1.radius - p2->radius << std::endl << std::flush;
                std::cout << "exactCollisionTime: " << eCT << std::endl << std::flush;
                std::cout << "p" << p1.id << " dt_left: " << *p1.dt_left << std::endl << std::flush;

                Vector p2_v(*p2->v);
                if (! p2->stationary)
                {
#pragma omp critical
                    *p2->v -=
                        (*p2->r - *p1.r) * (
                            (*p2->v - *p1.v)
                                .dot(*p2->r - *p1.r)
                                / pow(p2->r->distance(*p1.r), 2)
                                * 2 * p1.m / (p1.m + p2->m)
                        );
#pragma omp critical
                    *p2->v *= (1. - Settings::FORCES_DAMPEN_WATER);
                }
                if (! p1.stationary)
                {
                    *p1.v -=
                        (*p1.r - *p2->r) * (
                            (*p1.v - p2_v)
                                .dot(*p1.r - *p2->r)
                                / pow(p1.r->distance(*p2->r), 2)
                                * 2 * p2->m / (p1.m + p2->m)
                        );
                    *p1.v *= (1. - Settings::FORCES_DAMPEN_WATER);
                }

//                if (! p1.stationary)  *p1.r_former  = *p1.r;
//                if (! p2->stationary) *p2->r_former = *p2->r;
            }
        }
        former_p1_dt_left = *p1.dt_left;
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
