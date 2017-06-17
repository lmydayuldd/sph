#ifndef OPERATIONS_H
#define OPERATIONS_H

#include <Qvector4D>

#include "physics/geometry.h"

namespace Op {
    double sgn(double x);
}

void divideByW(QVector4D& v);
Geometry::Ray convertNormalized2DPointToRay(float normalizedX, float normalizedY);
double distanceBetween(const Vector& point, const Geometry::Ray& ray);
bool intersects(const Geometry::Sphere& sphere, const Geometry::Ray& ray);
Vector intersectionPoint(const Geometry::Ray& ray, const Geometry::Plane& plane);

#endif // OPERATIONS_H
