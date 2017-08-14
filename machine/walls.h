#ifndef WALLS_H
#define WALLS_H

class Particle;

#include "machine/machine.h"
#include "machine/obstacle.h"

class Walls : public Machine {
private:
    float lim;
    float damping;

public:
    Walls(float lim);

    void createView() override;
    void setModelMatrix() override;
    void paint() override;
    void collide(Particle *p2) override;
};

#endif // WALLS_H
