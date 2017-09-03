#ifndef SPHERE_H
#define SPHERE_H

#include "shape/shape.h"

class Sphere : public Shape {
private:
    std::vector<float> center;
    int smoothness;
    double radius;
    ShapeType type;

public:
    Sphere(double radius, float color[], ShapeType type);

    virtual void makeForm() override;
};

#endif // SPHERE_H
