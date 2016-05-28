#ifndef CLOTH_H
#define CLOTH_H

class Vector;

#include "machine/machine.h"

class Cloth : public Machine {
private:
    std::vector<Particle> flow;
    int knots;
    float color[3];

public:
    Cloth(
        const Vector& start,
        const Vector& end,
        int knots,
        float ks,
        float d,
        float kd,
        int strength
    );
    ~Cloth();

    void move();
    virtual void createView() override;
    virtual void setModelMatrix() override;
    virtual void paint() override;
    virtual void collide(Particle* p2) override;
};

#endif // CLOTH_H
