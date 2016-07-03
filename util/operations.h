#ifndef OPERATIONS_H
#define OPERATIONS_H

#include <iostream>

#include <Qvector4D>

#include "gl/matrices.h"
#include "physics/geometry.h"
#include "physics/vector.h"

void divideByW(QVector4D& v)
{
    for (int i = 0; i < 3; ++i)
    {
        v[i] /= v[3];
    }
}

Geometry::Ray convertNormalized2DPointToRay(float normalizedX, float normalizedY)
{
    // We'll convert these normalized device coordinates into world-space coordinates.
    // We'll pick a source on the near and far planes, and draw a line between them.
    // To do this transform, we need to first multiply by the inverse matrix, and then we need to undo the perspective divide.
    QVector4D nearPointNdc(normalizedX, normalizedY, -1, 1);
    QVector4D farPointNdc(normalizedX, normalizedY, 1, 1);
    QVector4D nearPointWorld = Matrices::viewProjectionInverted * nearPointNdc;
    QVector4D farPointWorld = Matrices::viewProjectionInverted * farPointNdc;
    divideByW(nearPointWorld);
    divideByW(farPointWorld);

    Vector nearPointRay
            = Vector(nearPointWorld[0], nearPointWorld[1], nearPointWorld[2]);
    Vector farPointRay
            = Vector(farPointWorld[0],  farPointWorld[1],  farPointWorld[2]);
    Vector ray = farPointRay - nearPointRay;
    return Geometry::Ray(nearPointRay, ray);
}

double distanceBetween(const Vector& point, const Geometry::Ray& ray)
{
    Vector p1ToPoint = point - ray.source;
    Vector p2ToPoint = point - (ray.source + ray.vector);
    // The length of the cross product gives the area
    // of an imaginary parallelogram having the two vectors as sides.
    // A parallelogram can be thought of as consisting of two triangles, so this
    // is the same as twice the area of the triangle defined by the two vectors.
    // http://en.wikipedia.org/wiki/Cross_product#Geometric_meaning
    double areaOfTriangleTimesTwo = (p1ToPoint * p2ToPoint).norm();
    double lengthOfBase           = ray.vector.norm();
    // The area of a triangle is also equal to (base * height) / 2.
    // In other words, the height is equal to (area * 2) / base.
    // The height of this triangle is the distance from the source to the ray.
    return areaOfTriangleTimesTwo / lengthOfBase;
}

bool intersects(const Geometry::Sphere& sphere, const Geometry::Ray& ray)
{
    return distanceBetween(sphere.center, ray) < sphere.radius;
}

Vector intersectionPoint(const Geometry::Ray& ray, const Geometry::Plane& plane)
{
    Vector rayToPlaneVector = plane.point - ray.source;
    double scaleFactor = rayToPlaneVector.dotProduct(plane.normal)
                        / ray.vector.dotProduct(plane.normal);
    Vector intersectionPoint = ray.source + ray.vector * scaleFactor;
    return intersectionPoint;
}

#endif // OPERATIONS_H
