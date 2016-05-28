#ifndef MACHINE_H
#define MACHINE_H

#include <iostream>
#include <vector>

//class Form;
#include "gl/form.h"
class Particle;

class Machine
{
public:
    static std::vector<Machine*> machines;
    Form* form;

    Machine();
    virtual ~Machine();

    void linkView(ShapeType formShapeName);
    void recolor(float color[]);
    virtual void setModelMatrix();
    virtual void paint();
    virtual void createView();
    virtual void collide(Particle* p2) = 0;
};

#endif // MACHINE_H
