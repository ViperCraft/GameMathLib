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