#ifndef TESTS_H
#define TESTS_H

#include "gtest/gtest.h"

#include "unit_tests/mock/mock_particle.h"
#include "unit_tests/mock/mock_vector.h"

class Tests : public ::testing::Test
{
protected:
    MockParticle* mockParticle;
    //MockVector* mockVector;

    // You can do set-up work for each test here.
    Tests()
    {
        mockParticle = new MockParticle();
        //mockVector = new MockVector();
    }

    // You can do clean-up work that doesn't throw exceptions here.
    virtual ~Tests()
    {
        delete mockParticle;
        //delete mockVector;
    }

    // Code here will be called immediately after the constructor
    // (right before each test).
    virtual void SetUp()
    {}

    // Code here will be called immediately after each test
    // (right before the destructor).
    virtual void TearDown()
    {}
};

#endif // TESTS_H
