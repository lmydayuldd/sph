#include "tests.h"

using ::testing::AtLeast;

TEST_F(Tests, ifItWorks)
{
    EXPECT_CALL(*mockParticle, paint())
            .Times(AtLeast(1));

    mockParticle->paint();
}
