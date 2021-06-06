# GameMathLib
C++ Vector Math Library for games, useful for physics, AI. Can be used in mobile phone games, its simple to use and fast!

* Vectors at any static sizes, 2D, 3D, 4D, 9D
* Matrix4 useful for 3D worlds, OpenGL matrix layout
* Quaternions
* Fuzzy logic stuff
* Mini and fast NNet

## Examples

How to build & run tests:
```shell
cmake . && make
./test/unit/unittest
```

## Basic Vector usage

```c++
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

```