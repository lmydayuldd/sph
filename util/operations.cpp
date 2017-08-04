#include "util/operations.h"

#include <iostream>

#include "gl/matrices.h"
#include "physics/vector.h"

namespace Op {
    double sgn(double x)
    {
        auto l = [](double x) -> double { return x < 0 ? -1 : 1; };
        return l(x);
    }
}

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
    double scaleFactor = rayToPlaneVector.dot(plane.normal)
                        / ray.vector.dot(plane.normal);
    Vector intersectionPoint = ray.source + ray.vector * scaleFactor;
    return intersectionPoint;
}

double lineLineMinDist(const Geometry::Line& l1, const Geometry::Line& l2)
{
    Vector u = l1.p2 - l1.p1;
    Vector v = l2.p2 - l2.p1;
    Vector w = l1.p1 - l2.p1;
    double a = u.dot(u); // always >= 0
    double b = u.dot(v);
    double c = v.dot(v); // always >= 0
    double d = u.dot(w);
    double e = v.dot(w);
    double D = a*c - b*b; // always >= 0
    double sc, tc;

    if (D < 0.000001) { // lines almost parallel
        sc = 0.;
        tc = (b>c ? d/b : e/c); // largest denominator
    }
    else {
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
    }

    // diff of two closest points
    Vector dP = w + u*sc- v*tc; // L1(sc) - L2(tc)
    return dP.norm();
}

bool lineIntersectsLine(const Geometry::Line& l1, const Geometry::Line& l2)
{
    bool intersects = false;
    if (lineLineMinDist(l1, l2) == 0.)
    {
        intersects = true;
    }
    return intersects;
}

double segSegMinDist(const Geometry::Line& l1, const Geometry::Line& l2)
{
    Vector u = l1.p2 - l1.p1;
    Vector v = l2.p2 - l2.p1;
    Vector w = l1.p1 - l2.p1;
    double a = u.dot(u); // always >= 0
    double b = u.dot(v);
    double c = v.dot(v); // always >= 0
    double d = u.dot(w);
    double e = v.dot(w);
    double D = a*c - b*b; // always >= 0
    double sc, sN, sD = D; // sc = sN/sD, default sD = D >= 0
    double tc, tN, tD = D; // tc = tN/tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < 0.000001) { // the lines are almost parallel
        sN = 0.; // force using point P0 on segment S1
        sD = 1.; // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else { // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.) { // sc < 0 => the s=0 edge is visible
            sN = 0.;
            tN = e;
            tD = c;
        }
        else if (sN > sD) { // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) { // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0) {
            sN = 0.0;
        }
        else if (-d > a) {
            sN = sD;
        }
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0) {
            sN = 0;
        }
        else if ((-d + b) > a) {
            sN = sD;
        }
        else {
            sN = (-d +  b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (abs(sN) < 0.000001 ? 0.0 : sN / sD);
    tc = (abs(tN) < 0.000001 ? 0.0 : tN / tD);

    // get the difference of the two closest points
    Vector dP = w + u*sc - v*tc; // S1(sc) - S2(tc)

    return dP.norm(); // return the closest distance
}

bool segIntersectsSeg(const Geometry::Line& l1, const Geometry::Line& l2)
{
    bool intersects = false;
    if (segSegMinDist(l1, l2) == 0.)
    {
        intersects = true;
    }
    return intersects;
}

double timePointPointClosest(const Vector& r1, const Vector& v1, const Vector& r2, const Vector& v2)
{
    Vector dv = v1 - v2;

    double dv2 = v1.dot(v2);
    if (dv2 < 0.000001)
    {
        return 0.;
    }

    Vector w0 = r1 - r2;
    double cpa_time = - w0.dot(dv) / dv2;

    return cpa_time;
}

double distPointPointClosest(const Vector& r1, const Vector& v1, const Vector& r2, const Vector& v2)
{
    double ctime = timePointPointClosest(r1, v1, r2, v2);
    Vector p1 = r1 + v1 * ctime;
    Vector p2 = r2 + v2 * ctime;

    return p1.distance(p2);
}
