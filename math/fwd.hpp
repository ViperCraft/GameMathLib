
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

#ifndef GAME_MATH_FWD_HPP
#define GAME_MATH_FWD_HPP

namespace nMath
{
    typedef unsigned int    IT;
    typedef unsigned char   UByte;

    namespace nInternal
    {
        template<typename T>
        struct do_nothing_init_policy;

        template<typename T>
        struct zero_init_policy;
    } // namespace nInternal

    //
    // forward declaration of types
    //
    // for most operation T must be numerical type
    //
    template
    <
        typename T,
        IT N,
        template<class> class IPolicy = nInternal::do_nothing_init_policy
    >                             class Vector;
    template<typename T>          class Matrix4; // represents matrix 4x4
    template<typename T>          class Quat;
    template<typename T>          class Plane;
    template<typename T, IT N>    class Face;

    typedef Vector<float, 2>  /*__attribute__((aligned(8)))*/    Vector2f;
    typedef Vector<float, 3>      Vector3f;
    typedef Vector<float, 4>      Vector4f;
    typedef Vector<int, 2>        Vector2i;
    typedef Vector<int, 3>        Vector3i;
    typedef Vector<IT, 2>         Vector2u;
    typedef Vector<UByte, 3>      Vector3ub;
    typedef Vector<UByte, 4>      Vector4ub;
    typedef Vector<int, 4>        Vector4i;
    typedef Vector<double, 2>     Vector2d;
    typedef Vector<double, 3>     Vector3d;
    typedef Vector<double, 4>     Vector4d;
    typedef Vector<float, 8>      Vector8f;
    typedef Vector<float, 6>      Vector6f;

    typedef Matrix4<float>        Matrix4f;
    typedef Matrix4<double>       Matrix4d;

    typedef Quat<float>           Quatf;

    typedef Plane<float>          Planef;

    typedef Face<float, 3>        Face3f;
} // namespace nMath

#endif // !GAME_MATH_FWD_HPP
