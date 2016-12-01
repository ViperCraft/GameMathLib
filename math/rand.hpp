
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

#include <cstdlib>
#include <cassert>

namespace nRandomize
{
    // based on IEEE numeric standards ( dancing with bits )    
    namespace nInternal
    {
        template<typename T> struct frand_impl;
        template<> struct frand_impl<float>
        {
            static float Do( unsigned int uRand )
            {
                static_assert( sizeof(float) == 4 && sizeof(int) == 4, "float and int sizes must be equal" );
                enum { jflone = 0x3f800000U, jflmsk = 0x007fffffU };
                
                union
                {
                    unsigned int i;
                    float f;
                } pun;
                
                pun.i = jflone | (uRand & jflmsk);
                return pun.f - 1.0f;
            }
        };
    }
    // template only need for 'definition linkage'
    template<int>
    class FastRandom
    {
    public:
        static void seed( unsigned int uSeed )
        {
            iran = uSeed;
        }
        // return range 0..0xffffffff
        static unsigned int irand()
        {
            return iran = 1664525 * iran + 1013904223;
        }
        // return range 0..1
        template<typename T>
        static T frand()
        {
            return nInternal::frand_impl<T>::Do(irand());
        }
    private:
        static unsigned int iran;
    };

    template<int value>
    unsigned int FastRandom<value>::iran = 0;

    class FastRandomHandle
    {
    public:
        FastRandomHandle( unsigned int uSeed = 0 )
            :
            iran(uSeed)
            {}
        void seed( unsigned int uSeed )
        {
            iran = uSeed;
        }
        // return range 0..0xffffffff
        unsigned int irand()
        {
            return iran = 1664525 * iran + 1013904223;
        }
        // return range 0..1
        template<typename T>
        T frand()
        {
            return nInternal::frand_impl<T>::Do(irand());
        }
        float operator() ()
        {
            return frand<float>();
        }
    private:
        unsigned int iran;
    };
    
    class MarsagliaRandomHandle
    {
    public:
        MarsagliaRandomHandle( unsigned int uSeed = 0 )
        { seed(uSeed); }
        
        void seed( unsigned int uSeed )
        {
            x = uSeed;
            y = 842502087U;
            z = 3579807591U;
            w = 273326509U;
        }
        // return range 0..0xffffffff
        unsigned int irand()
        {
            unsigned int t=(x^(x<<11));
			x=y; y=z; z=w;
			w=(w^(w>>19))^(t^(t>>8));
            return w;
        }
        // return range 0..1
        float frand()
        {
            union { uint32_t i; float f; } u;	// union for floating point conversion of result
            u.i = 0x3F800000 | (irand() >> 9);  // 0-23 bit mantissa
            return u.f - 1.f;
        }
        float operator() ()
        {
            return frand();
        }
    private:
        unsigned int x, y, z, w;
    };
    
    struct StdRandom
    {
        static void seed( unsigned int uSeed )
        {
            srand(uSeed);
        }
        // return range 0..0xffffffff
        static unsigned int irand()
        {
            return static_cast<unsigned int>(::rand());
        }
        // return range 0..1
        template<typename T>
        static T frand()
        {
            return static_cast<T>(rand()) / RAND_MAX;
        }
    };
} // namespace nRandomize

namespace nMath
{
    typedef nRandomize::StdRandom       RandT;
    
    template<typename T> T randReasonable()
    {
        return static_cast<T>(.1) + static_cast<T>(.2) * RandT::frand<T>();
    }
    
    template<typename T> T randMinMax( T const lhs, T const rhs )
    {
        return lhs + (rhs - lhs) * RandT::frand<T>();
    }
    
    template
    <
        typename T,
        typename F
    >
    T randMinMax( T const lhs, T const rhs, F & f )
    {
        return lhs + (rhs - lhs) * f();
    }
    
    template<typename T> T randMinMaxFine( T const lhs, T const rhs )
    {
        return (lhs - cNearHalf<T>()) + RandT::frand<T>() * (rhs - lhs + cNearOne<T>());
    }
    
    template<typename T> T randMinMax( T const minMax )
    {
        return minMax * cTwo<T>() * RandT::frand<T>() - minMax;
    }
    
    template<typename T, typename F> T randMinMax( T const minMax, F &f )
    {
        return minMax * cTwo<T>() * f() - minMax;
    }
    
    
} // namespace nMath

