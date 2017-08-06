#ifndef SHAPE_H
#define SHAPE_H

#include <vector>

#include "gl/form.h"

class Shape
{
public:
    Form* form;
    std::vector<float> vertices;
    int posCoordsPerVertex = 0;
    int clrCoordsPerVertex = 0;

    virtual ~Shape();
    Shape();

    virtual void makeForm() = 0;
};

#endif // SHAPE_H
