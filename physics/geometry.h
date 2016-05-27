#ifndef GEOMETRY_H
#define GEOMETRY_H

class Vector;

namespace Geometry {
    class Ray;
    class Sphere;
    class Plane;
}

class Geometry::Ray {
public:
    Vector *source, *vector;
    Ray(Vector *source, Vector *vector) {
        this->source = source;
        this->vector = vector;
    }
};

class Geometry::Sphere {
public:
    Vector* center;
    double radius;
    Sphere(Vector* center, double radius) {
        this->center = center;
        this->radius = radius;
    }
};

class Geometry::Plane {
public:
    Vector *point, *normal;
    Plane(Vector* point, Vector* normal) {
        this->point  = point;
        this->normal = normal;
    }
};

#endif // GEOMETRY_H
