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

#ifndef GAME_MATH_IOS_HPP
#define GAME_MATH_IOS_HPP

#include <ostream>

namespace nMath
{
    template
    <
        typename              T,
        unsigned              N,
        template<class> class IP
    >
    std::ostream& operator << ( std::ostream& os, Vector<T, N, IP> const& vec )
    {
        os << '{' << vec[0];
        for( IT i = 1; i < N; i++ )
            os << ", " << vec[i];
        os << '}';
        return os;
    }
    
    template
    <
        unsigned              N,
        template<class> class IP
    >
    std::ostream& operator << ( std::ostream& os, Vector<unsigned char, N, IP> const& vec )
    {
        os << '{' << int(vec[0]);
        for( IT i = 1; i < N; i++ )
            os << ", " << int(vec[i]);
        os << '}';
        return os;
    }

    template<typename T>
    std::ostream& operator << ( std::ostream& os, Matrix4<T> const& mat )
    {
        os << '[' << mat.get_values()[0];
        for( IT i = 1; i < 16; i++ )
            os << ", " << mat.get_values()[i];
        os << ']';
        return os;
    }

    template<typename T>
    std::ostream& operator << ( std::ostream& os, Quat<T> const& q )
    {
        os << '(' << q.get_values()[0];
        for( IT i = 1; i < 4; i++ )
            os << ", " << q.get_values()[i];
        os << ')';
        return os;
    }
} // namespace nMath

#endif // !GAME_MATH_IOS_HPP
