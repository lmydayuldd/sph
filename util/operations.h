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

double lineLineMinDist(const Geometry::Line& l1, const Geometry::Line& l2);
bool lineIntersectsLine(const Geometry::Line& l1, const Geometry::Line& l2);
double segSegMinDist(const Geometry::Line& l1, const Geometry::Line& l2);
bool SegIntersectsSeg(const Geometry::Line& l1, const Geometry::Line& l2);
double timePointPointClosest(const Vector& r1, const Vector& v1, const Vector& r2, const Vector& v2);
double distPointPointClosest(const Vector& r1, const Vector& v1, const Vector& r2, const Vector& v2);

#endif // OPERATIONS_H
