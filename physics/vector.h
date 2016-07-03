#ifndef VECTOR_H
#define VECTOR_H

#include "machine/machine.h"

#include <cmath>
#include <ostream>

class Vector : public Machine {
public:
    double x, y, z;
    Vector* source;

    ~Vector();
    Vector();
    Vector(double x, double y, double z);
    Vector(const Vector& v); // copy constructor

    friend std::ostream& operator<<(std::ostream& out, const Vector& v)
    {
        return out << v.x << " " << v.y << " " << v.z << std::endl;
    }

    void createView();

    bool operator==(const Vector& v) const;
    Vector operator+(const Vector& v) const;
    Vector operator-(const Vector& v) const;
    Vector operator*(const Vector& v) const; // crossProduct
//    Vector operator/(const Vector& v) const;
    Vector operator+(double v) const;
    Vector operator-(double v) const;
    Vector operator*(double v) const;
    Vector operator/(double v) const;
    Vector operator-() const;

    double      cosxy(const Vector& v) const;
    double   distance(const Vector& v) const;
    double dotProduct(const Vector& v) const;
//    Vector crossProduct(const Vector& v) const;

    double   norm() const;
    Vector normal() const;

    void setApplicationPoint(Vector application);
    void setModelMatrix() override;
    void paint() override;
    void collide(Particle* p2) override;
};

#endif // VECTOR_H
