#ifndef ROPE_H
#define ROPE_H

//class Flow;
#include "machine/flow.h"

class Vector;

class Rope
{
private:
    Flow* flow;

public:
    ~Rope();
    Rope(
        Vector* start,
        Vector* end,
        int knots,
        float ks,
        float d,
        float kd,
        int strength
    );

    void paint();
};

#endif // ROPE_H
