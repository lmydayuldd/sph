#ifndef ROPE_H
#define ROPE_H

#include "machine/particle.h"
#include "physics/vector.h"

#include <vector>

class Rope : public Machine
{
private:
    std::vector<Particle*>* flow;

public:
    ~Rope();
    Rope(
        const Vector& start,
        const Vector& end,
        int knots,
        int strength,
        float ks,
        float d,
        float kd
    );

    virtual void paint() override;
};

#endif // ROPE_H
