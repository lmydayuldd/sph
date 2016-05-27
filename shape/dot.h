#ifndef DOT_H
#define DOT_H

#include "shape/shape.h"

class Dot : public Shape {
private:
    float color[3];

    void makeForm();

public:
    Form* form;

    ~Dot();
    Dot();
    Dot(float color[]);
};

#endif // DOT_H
