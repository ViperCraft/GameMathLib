#include "../../math/math.hpp"
#include "../../math/fuzzy.hpp"
#include <algorithm>
#include "gtest/gtest.h"
using namespace nMath;

TEST(VectorsBasic, TestIsTrue)
{
    Vector4f vec4; // vector dimeonsion 4 of floats
    Vector3d vec3; // vector dimension 3 of doubles

    vec4.x() = 1;
    vec4.y() = 2;
    vec4[2]  = 3;
    vec4.w() = 4;

    vec3[0]  = 1;
    vec3[1]  = 2;

    // access at index at compile time  
    vec3.at_c<2>() = 3;

    Vector4d v4_cpy(vec4);

    vec4 += v4_cpy;

    EXPECT_EQ( vec4, Vector4f(2, 4, 6, 8) );

    vec4 -= Vector4i(v4_cpy);

    EXPECT_EQ( vec4, Vector4f(1, 2, 3, 4) );

    Vector<float, 100> feature_vector;

    std::generate(feature_vector.begin(), feature_vector.end(), std::rand);

    feature_vector.normalize();

    EXPECT_EQ( true, is_near(feature_vector.magnitude(), 1.f) );
}

static int fibonachi( int v )
{
    int fib = 1;
    if( v > 1 )
        fib = fibonachi(v - 1);
    return v * fib;
}

TEST(ZeroInitTest, TestIsTrue)
{
    // call some recursion here to trash stack memory
    fibonachi(256);
    // by default Vector<T> has no defaut initialization
    Vector3f vec3_def;
    Vector<float, 3, nInternal::zero_init_policy> vec3_zero;
    
    char zeroed_mem[ sizeof(Vector3f) ] = { 0 };

    /// must be zero in vec3_zero
    /// and vec3_def has some random data
    EXPECT_EQ( 0, memcmp(vec3_zero.begin(), zeroed_mem, sizeof(zeroed_mem)) );
    // and this vector with default init must be non-zero
    ASSERT_NE( 0, memcmp(vec3_def.begin(), zeroed_mem, sizeof(zeroed_mem)) );

}

TEST(FuzzyLogic, TestIsTrue)
{
    fuzzyd a(1.), b(0.5);

    EXPECT_EQ( false, f_true(a && b));
    EXPECT_EQ( true,  f_true(a || b));

    b.normalize();

    EXPECT_EQ( true, f_true(a && b));
    EXPECT_EQ( true,  f_true(a || b));
}

TEST(MatrixVsQuatIdentity, TestIsTrue)
{
    Matrix4f mat1, mat2; // identity

    Quatf q1, q2;
    q1.to_matrix2(mat2); // convert identity quaternion to mat4

    q2.from_matrix(mat1);

    EXPECT_EQ( true, q1.is_equal(q2) );
}

TEST(MatrixTranspose, TestIsTrue)
{
    Matrix4<int> mat1;
    int v = 1;
    for( auto &e : mat1 )
    {
        e = v++;
    }
    auto mat2 = mat1.get_transpose();

    // check column major order
    EXPECT_EQ( true, mat1.vFront() == Vector4i(1,   2,  3,  4) );
    EXPECT_EQ( true, mat1.vUp()    == Vector4i(5,   6,  7,  8) );
    EXPECT_EQ( true, mat1.vRight() == Vector4i(9,  10, 11, 12) );
    EXPECT_EQ( true, mat1.vPosit() == Vector4i(13, 14, 15, 16) );

    // check transposed row-major order
    EXPECT_EQ( true, mat2.vFront() == Vector4i(1, 5,  9, 13) );
    EXPECT_EQ( true, mat2.vUp()    == Vector4i(2, 6, 10, 14) );
    EXPECT_EQ( true, mat2.vRight() == Vector4i(3, 7, 11, 15) );
    EXPECT_EQ( true, mat2.vPosit() == Vector4i(4, 8, 12, 16) );

}