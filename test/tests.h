#ifndef TESTS_H
#define TESTS_H

#include "gtest/gtest.h"
#include "mock/mock_particle.h"

class Tests : public ::testing::Test
{
protected:
    ParticleMock* mockParticle;

    // You can do set-up work for each test here.
    Tests();

    // You can do clean-up work that doesn't throw exceptions here.
    virtual ~Tests();

    // Code here will be called immediately after the constructor
    // (right before each test).
    virtual void SetUp();

    // Code here will be called immediately after each test
    // (right before the destructor).
    virtual void TearDown();

    // Objects declared here can be used by all tests in the test case for Foo.
};

#endif // TESTS_H
