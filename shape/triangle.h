#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "shape/shape.h"

class Triangle : public Shape {
private:
    float side;

public:
    Triangle(float side);

    virtual void makeForm() override;
};

#endif // TRIANGLE_H
