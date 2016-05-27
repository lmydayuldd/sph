#ifndef CLOTH_H
#define CLOTH_H

class Flow;
class Vector;

#include "machine/machine.h"

class Cloth : public Machine {
private:
    Flow* flow;
    int knots;
    float color[3];

public:
    Cloth(
        Vector* start,
        Vector* end,
        int knots,
        float ks,
        float d,
        float kd/*,
        int strength*/
    );
    ~Cloth();

    void setModelMatrix() override;
    void paint() override;
};

#endif // CLOTH_H
