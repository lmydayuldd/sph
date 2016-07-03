#include "unit_tests/test/vector_test.h"

#include "physics/vector.h"

// =============================================================================
// == vector_op_vector tests ===================================================
// =============================================================================

TEST_F(VectorTest, vector_add_vector_test)
{
    Vector u = Vector(-3, 14, 7);
    Vector v = Vector(6, 12, -3.1);
    EXPECT_EQ(u + v, Vector(3, 26, 3.9));
}

TEST_F(VectorTest, vector_sub_vector_test)
{
    Vector u = Vector( 4,  2,   -3);
    Vector v = Vector(-1, -2.7, -0.1);
    EXPECT_EQ(u - v, Vector(5, 4.7, -2.9));
}

TEST_F(VectorTest, vector_mul_vector_test)
{
    Vector u = Vector(4, -2, -7);
    Vector v = Vector(1,  5, -1);
    //EXPECT_EQ(u * v, );
}

// =============================================================================
// == vector_op_scalar tests ===================================================
// =============================================================================

TEST_F(VectorTest, vector_add_scalar_test)
{

}

TEST_F(VectorTest, vector_sub_scalar_test)
{

}

TEST_F(VectorTest, vector_mul_scalar_test)
{

}

TEST_F(VectorTest, vector_div_scalar_test)
{

}

// =============================================================================
// == other vector ops tests ===================================================
// =============================================================================

TEST_F(VectorTest, cosxy_test)
{

}

TEST_F(VectorTest, distance_test)
{
    Vector u = Vector( 1,  1,  1);
    Vector v = Vector(-1, -1, -1);
    EXPECT_EQ(2 * sqrt(3), u.distance(v));
    EXPECT_EQ(2 * sqrt(3), v.distance(u));
}

TEST_F(VectorTest, dotProduct_test)
{
    Vector u = Vector( 1,  1,  1);
    Vector v = Vector(-1, -1, -1);
    EXPECT_EQ(1 * (-1) + 1 * (-1) + 1 * (-1), u.dotProduct(v));
    EXPECT_EQ(1 * (-1) + 1 * (-1) + 1 * (-1), v.dotProduct(u));
}

TEST_F(VectorTest, norm_test)
{
    Vector u = Vector(-1, -1, -1);
    EXPECT_EQ(sqrt(u.dotProduct(u)), u.norm());
    EXPECT_EQ(sqrt((-1) * (-1) + (-1) * (-1) + (-1) * (-1)), u.norm());
}

TEST_F(VectorTest, normal_test)
{

}
