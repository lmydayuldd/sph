#ifndef MACHINE_H
#define MACHINE_H

#include <iostream>
#include <vector>

class Form;
class Particle;

#include "util/enums.h"

class Machine
{
public:
    static std::vector<Machine*> machines;
    Form *form;

    Machine();
    virtual ~Machine();

    void linkView(ShapeType formShapeType);
    void recolor(float color[]);
    virtual void setModelMatrix() {}
    virtual void paint();
    virtual void createView() {}
    virtual void collide(Particle *p2) {} // = 0; // for pure virtual function
};

#endif // MACHINE_H
