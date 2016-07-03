#ifndef VECTOR_TEST_H
#define VECTOR_TEST_H

#include "gtest/gtest.h"

#include "unit_tests/mock/mock_particle.h"

class VectorTest : public ::testing::Test
{
protected:
    MockParticle* mockParticle;

    VectorTest()
    {
        mockParticle = new MockParticle();
    }

    virtual ~VectorTest()
    {
        delete mockParticle;
    }
};

#endif // VECTOR_TEST_H
