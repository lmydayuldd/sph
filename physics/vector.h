#ifndef VECTOR_H
#define VECTOR_H

#include "machine/machine.h"

#include <cmath>
#include <ostream>

class Vector : public Machine {
public:
    double x, y, z;
    Vector *source;

    ~Vector();
    Vector();
    Vector(double x, double y, double z);
    Vector(const Vector &v); // copy constructor

    friend std::ostream& operator<<(std::ostream &out, const Vector &v)
    {
        return out << v.x << " " << v.y << " " << v.z << std::endl;
    }

    void createView();

    virtual Vector operator-() const;
    virtual bool   operator==(const Vector &v) const;
    virtual Vector operator+(const Vector &v) const;
    virtual Vector operator-(const Vector &v) const;
    virtual Vector operator*(const Vector &v) const; // cross
    //virtual double operator/(const Vector& v) const; // dot
    virtual Vector operator+(double v) const;
    virtual Vector operator-(double v) const;
    virtual Vector operator*(double v) const;
    virtual Vector operator/(double v) const;
    virtual Vector& operator+=(const Vector &v);
    virtual Vector& operator-=(const Vector &v);
    virtual Vector& operator*=(const Vector &v);
    virtual Vector& operator+=(double v);
    virtual Vector& operator-=(double v);
    virtual Vector& operator*=(double v);
    virtual Vector& operator/=(double v);

    virtual double    cosxy(const Vector &v) const;
    virtual double distance(const Vector &v) const;
    virtual double      dot(const Vector &v) const;

    virtual double   norm() const;
    virtual Vector normal() const;

    void limit(double max);
    void zero();

    void setApplicationPoint(Vector application);
    void setModelMatrix() override;
    void paint() override;
};

#endif // VECTOR_H
