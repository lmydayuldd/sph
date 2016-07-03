#include "control/interaction.h"

#include <typeinfo>

#include "machine/particle.h"
#include "physics/computer.h"
#include "physics/geometry.h"
#include "util/operations.h"

Particle* Interaction::pressedParticle = nullptr;
Vector* Interaction::lockedPressedParticlePosition = nullptr;

void Interaction::handleTouchPress(float normalizedX, float normalizedY)
{
    Geometry::Ray ray = convertNormalized2DPointToRay(normalizedX, normalizedY);
//    if (Computer::currentComputer != nullptr) {
        /*for (Flow f : Flow.flows) for (Particle p : f.particles) {
            // Now test if this ray intersects with the mallet
            // by creating a bounding sphere that wraps the mallet.
            Sphere particleBoundingSphere = new Sphere( new Vector( p.r ), p.radius );
            if ( intersects(particleBoundingSphere, ray) ) {
                pressedParticle = p;
                break;
            }
        }*/

        double minProximity = std::numeric_limits<double>::infinity();
        Particle* closest = nullptr;
        for (unsigned int i = 0; i < Particle::flows.size(); ++i)
        {
            for (Particle* p : Particle::flows[i])
            {
                Geometry::Sphere particleBoundingSphere
                        = Geometry::Sphere(*p->r, p->radius);
                double distance
                        = distanceBetween(particleBoundingSphere.center, ray);
                if (distance < minProximity)
                {
                    minProximity = distance;
                    closest = p;
                }
            }
        }
//        if ( minProximity < 0.1 ) {
            delete pressedParticle;
            pressedParticle = closest;
//        }
//    }
//    if (typeid(Computer::currentComputer) == typeid(Wave2DComputer)) {
//        double minProximity = std::numeric_limits<double>::infinity();
//        vector<Shape> shapes = Wave2DComputer.landschaft.shapes;
//        int x = 0, y = 0;
//        for (auto s = begin(shapes); s != end(shapes); ++s) {
//            for (unsigned int j = 0; j < (*s).posCoords.size(); j += 24) { ////////////////////////////////////////
//                Sphere quadrantBoundingSphere = Sphere(
//                    Vector((*s).posCoords[j], (*s).posCoords[j+1], (*s).posCoords[j+2]),
//                    Computer::currentComputer->dx
//                );
//                double distance = distanceBetween(quadrantBoundingSphere.center, ray);
//                if (distance < minProximity) {
//                    minProximity = distance;
//                    x = s - begin(shapes);
//                    y = j;
//                }
//            }
//        }
//        Computer::currentComputer->flagUp(x, y / 12, Wave2DComputer.CELL_BND);
//    }
}

void Interaction::handleTouchDrag(float normalizedX, float normalizedY)
{
    Geometry::Ray ray = convertNormalized2DPointToRay(normalizedX, normalizedY);
    //if (typeid(Computer::currentComputer) == typeid(ParticleComputer)) {
        if (pressedParticle != nullptr)
        {
            // Define a plane 1. parallel to default view plane
            // and 2. crossing our Particle
            Vector pressedParticlePosition = Vector(*pressedParticle->r);
            Vector straightVector = Vector(0, 0, 1);
            Geometry::Plane plane
                   = Geometry::Plane(pressedParticlePosition, straightVector);
            // Find out where the touched point intersects the
            // aforementioned plane. We'll move the particle along it.
            Vector touchedPoint = intersectionPoint(ray, plane);
            delete lockedPressedParticlePosition;
            lockedPressedParticlePosition
                = new Vector(touchedPoint.x, touchedPoint.y, pressedParticle->r->z);
            holdPressedParticle();
        }
    //}
}

void Interaction::holdPressedParticle()
{
    if (pressedParticle != nullptr)
    {
        if (lockedPressedParticlePosition != nullptr)
        {
            *pressedParticle->r = *lockedPressedParticlePosition;
            *pressedParticle->v = Vector();
        }
    }
}

void Interaction::handleTouchDrop()
{
    pressedParticle = nullptr;
    lockedPressedParticlePosition = nullptr;
}
