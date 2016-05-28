#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape/shape.h"

class Rectangle : public Shape {
public:
    Rectangle(float c[]/*oords*//*x1 x2 x3 y1 y2*/, float color[]);
    Rectangle(float lim, float color[]);

    void move(
        float x1, float y1,
        float x2, float y2,
        float x3, float y3,
        float x4, float y4,
        float x5, float y5
    );
    virtual void makeForm() override;
};

#endif // RECTANGLE_H
