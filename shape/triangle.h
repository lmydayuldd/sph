#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "shape/shape.h"

class Triangle : public Shape {
private:
    float side;

public:
    Triangle(float side);

    void makeForm();
};

#endif // TRIANGLE_H
