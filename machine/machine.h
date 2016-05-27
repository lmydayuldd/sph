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
    static std::vector<Machine> machines;
    Form* form;

    Machine();
    virtual ~Machine();

    void linkView(ShapeNames formShapeName);
    void recolor(float color[]);
    virtual void paint();
    virtual void createView();
    virtual void setModelMatrix();
    virtual void collide(Particle* p2);
};

#endif // MACHINE_H
