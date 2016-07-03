#ifndef MOCK_VECTOR_H
#define MOCK_VECTOR_H

#include "gmock/gmock.h"

#include "machine/particle.h"
#include "physics/vector.h"

class MockVector : public Vector
{
public:
    MOCK_CONST_METHOD1(cosxy, double(const Vector& v));
    MOCK_CONST_METHOD1(distance, double(const Vector& v));
    MOCK_CONST_METHOD1(dotProduct, double(const Vector& v));

    MOCK_CONST_METHOD0(norm, double());
    MOCK_CONST_METHOD0(normal, Vector());

    MOCK_METHOD1(setApplicationPoint, void(Vector application));
    MOCK_METHOD0(setModelMatrix, void());
    MOCK_METHOD0(paint, void());
    MOCK_METHOD1(collide, void(Particle* p2));
};

#endif // MOCK_VECTOR_H
