#ifndef SHAPE_H
#define SHAPE_H

#include "gl/form.h"

#include <vector>

class Shape
{
public:
    Form *form;
    std::vector<float> vertices;
    int posCoordsPerVertex = 0;
    int clrCoordsPerVertex = 0;

    virtual ~Shape();
    Shape();

    virtual void makeForm() = 0;
};

#endif // SHAPE_H
