#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>

#include "physics/vector.h"

namespace Geometry {
    class Ray;
    class Sphere;
    class Plane;
    class Line;
}

class Geometry::Ray {
public:
    Vector source, vector;
    Ray(Vector& source, Vector& vector) {
        this->source = source;
        this->vector = vector;
    }
};

class Geometry::Sphere {
public:
    Vector center;
    double radius;
    Sphere(Vector& center, double radius) {
        this->center = center;
        this->radius = radius;
    }
};

class Geometry::Plane {
public:
    Vector point, normal;
    Plane(Vector& point, Vector& normal) {
        this->point  = point;
        this->normal = normal;
    }
};

class Geometry::Line {
public:
    Vector p1, p2;
    Line(Vector& p1, Vector& p2) {
        this->p1 = p1;
        this->p2 = p2;
    }
};

#endif // GEOMETRY_H
