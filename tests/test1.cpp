#include <assert.h>
#include <cstdlib>
#include <algorithm>
#include "../math/math.hpp"
#include "../math/utils.hpp"
#include "../math/ios.hpp"

int main()
{
  using namespace nMath;
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

  assert( vec4 == Vector4f(2, 4, 6, 8) );

  vec4 -= Vector4i(v4_cpy);

  assert( vec4 == Vector4f(1, 2, 3, 4) );

  Vector<float, 100> feature_vector;

  std::generate(feature_vector.begin(), feature_vector.end(), std::rand);

  feature_vector.normalize();

  assert( is_near(feature_vector.magnitude(), 1.f) );
  return 0;
}
