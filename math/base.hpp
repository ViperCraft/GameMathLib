/***************************************************************************
 *   Copyright (C) 2003-2008 by Sergey Kasak (aka Viper Craft)             *
 *   viper.craft@gmail.com                                                 *
 *                                                                         *
 *   All rights reserved.                                                  *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *   Redistributions of source code must retain the above copyright        *
 *   notice, this list of conditions and the following disclaimer.         *
 *   Redistributions in binary form must reproduce the above copyright     *
 *   notice, this list of conditions and the following disclaimer in the   *
 *   documentation and/or other materials provided with the distribution.  *
 *   Neither the name of the <ORGANIZATION> nor the names of its           *
 *   contributors may be used to endorse or promote products derived from  *
 *   this software without specific prior written permission.              *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR *
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, *
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      *
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, *
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY *
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   *
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE     *
 *   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH      *
 *   DAMAGE.                                                               *
 ***************************************************************************/
#pragma once

namespace nMath
{
    // constants converters
    template<typename T> constexpr T cBigEPSILON()   { return static_cast<T>(10e-3); }
    template<typename T> constexpr T cEPSILON()      { return static_cast<T>(10e-4); }
    template<typename T> constexpr T cSmallEPSILON() { return static_cast<T>(10e-6); }
    template<typename T> constexpr T cNearOne()      { return static_cast<T>(0.9999); }
    template<typename T> constexpr T cOne()          { return static_cast<T>(1.); }
    template<typename T> constexpr T cZero()         { return static_cast<T>(0.); }
    template<typename T> constexpr T cTwo()          { return static_cast<T>(2.); }
    template<typename T> constexpr T cThree()        { return static_cast<T>(3.); }
    template<typename T> constexpr T cFour()         { return static_cast<T>(4.); }
    template<typename T> constexpr T cFive()         { return static_cast<T>(5.); }
    template<typename T> constexpr T cPI()           { return static_cast<T>(3.1415926535897932384626433832795028841971); }
    template<typename T> constexpr T c2PI()          { return cTwo<T>() * cPI<T>(); }
    template<typename T> constexpr T cToRad()        { return static_cast<T>(cPI<double>() / 180.0); }
    template<typename T> constexpr T cHalf()         { return static_cast<T>(.5); }
    template<typename T> constexpr T cQuarter()      { return static_cast<T>(.25); }
    template<typename T> constexpr T cNearHalf()     { return static_cast<T>(.499999); }
    template<typename T> constexpr T cDeg2RadConst() { return static_cast<T>(cPI<double>() / 180.0); }
    template<typename T> constexpr T cRad2DegConst() { return static_cast<T>(180.0 / cPI<double>()); }

    //
    // compile time functions
    //
    namespace meta {

    template<IT N, IT M>
    struct smin
    {
        enum { result = (N < M) ? N : M };
    };

    template<IT N, IT M>
    struct smax
    {
        enum { result = (N > M) ? N : M };
    };

    template<int a, int times>
    struct mul
    {
        enum { value = a * mul<a, times - 1>::value };
    };

    template<int a>
    struct mul<a, 0>
    {
        enum { value = 1 };
    };

    } // namespace meta

    template<bool v>
    struct bool2value
    {
        enum { value = v };
    };

    //
    // runtime functions
    //

    template<typename T>
    T sqr( T const v )
    {
        return v * v;
    }

    // return static array size
    template<typename T, size_t N>
    size_t array_size( T (&)[N] )
    {
        return N;
    }

    template<typename T>
    bool is_near
    (
        T const value,
        T const target,
        T const accuracy = cSmallEPSILON<T>()
    )
    {
        return 
            value == target
            ||
            (
                (value + accuracy) >= target
                &&
                (value - accuracy) <= target
            )
        ;
    }


    // basic functions
    template<typename T> T deg2rad( T const angle )    { return cDeg2RadConst<T>() * angle; }
    template<typename T> T rad2deg( T const rad )      { return cRad2DegConst<T>() * rad;   }
    template<typename T> T reciprocal( T const arg )   { return cOne<T>() / arg;            }
    template<typename T> T reciprocalSQRT( T const arg )
    {
        return reciprocal(std::sqrt(arg));
    }

    // linear interpolation
    template<typename T> T lerp( T const a, T const b, T const t ) 
    {
        return t * ( b - a ) + a;
    }

    template<typename T> T wrap( T const x, T const lhs, T const rhs )
    {
        return (x < lhs) ? (x - lhs) + rhs : (x > rhs) ? (x - rhs) + lhs : x;
    }

    template<typename T> T clamp( T const value, T const lhs, T const rhs )
    {
        return rhs < value ? rhs : lhs > value ? lhs : value;
    }
} // namespace nMath

