#include "tests.h"

using ::testing::AtLeast;

Tests::Tests()
{
    mockParticle = new ParticleMock();
}

Tests::~Tests()
{
    delete mockParticle;
}

void Tests::SetUp() {}
void Tests::TearDown() {}

TEST_F(Tests, ifItWorks)
{
    EXPECT_CALL(*mockParticle, paint())
            .Times(AtLeast(1));

    mockParticle->paint();
}
