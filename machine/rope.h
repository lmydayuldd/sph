#ifndef ROPE_H
#define ROPE_H

#include <vector>

#include "machine/particle.h"
#include "physics/vector.h"

class Rope : public Machine
{
private:
    std::vector<Particle> flow;

public:
    ~Rope();
    Rope(
        const Vector& start,
        const Vector& end,
        int knots,
        float ks,
        float d,
        float kd,
        int strength
    );

    virtual void paint() override;
    virtual void collide(Particle* p2) override;
};

#endif // ROPE_H
