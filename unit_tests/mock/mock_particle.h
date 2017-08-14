#ifndef MOCK_PARTICLE_H
#define MOCK_PARTICLE_H

#include "gmock/gmock.h"

#include "machine/particle.h"

class MockParticle : public Particle
{
public:
    //MOCK_METHOD0(Particle, void());
    //MOCK_METHOD0(Particle, void(int parentFlow));
    MOCK_METHOD0(Die, void());
    virtual ~MockParticle() { Die(); }

    MOCK_METHOD4(springify,
                 void(Particle *p2, float ks, float d, float kd));
    MOCK_METHOD4(springifyMutual,
                 void(Particle *p2, float ks, float d, float kd));
    MOCK_METHOD0(createView, void());
    MOCK_METHOD0(setModelMatrix, void());
    MOCK_METHOD0(paint, void());
    MOCK_METHOD1(collide, void(Particle*));
};

#endif // MOCK_PARTICLE_H
