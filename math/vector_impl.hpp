
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

#ifndef GAME_MATH_VECTOR_IMPL_HPP
#define GAME_MATH_VECTOR_IMPL_HPP

#include "base.hpp"

namespace nMath
{
    namespace nInternal
    {
        template<typename T, IT N>
        struct cross_prod
        {
            static void Do( T*, T const*, T const* ) {}
        };

        // cross product defined only for 3D vectors!
        template<typename T>
        struct cross_prod<T, 3>
        {
            static void Do( T *out, const T *a, const T *b )
            {
                out[0] = a[1] * b[2] - a[2] * b[1];
                out[1] = a[2] * b[0] - a[0] * b[2];
                out[2] = a[0] * b[1] - a[1] * b[0];
            }
        };

        template<typename T, typename U, IT N>
        struct cast_op
        {
            static void Do( T *dest, const U *src )
            {
                *dest = static_cast<T>(*src);
                cast_op<T, U, N-1>::Do(++dest, ++src);
            }
        };

        template<typename T, typename U>
        struct cast_op<T, U, 0>
        {
            static void Do( T*, const U* ) {}
        };

        template<typename T, IT N>
        struct neg_op
        {
            static void Do( const T *in, T *out )
            {
                *out = -(*in);
                neg_op<T, N-1>::Do(++in, ++out);
            }
        };
        
        template<typename T>
        struct sqr_diff
        {
            T operator () ( T const lhs, T const rhs ) const
            {
                T const diff = lhs - rhs;
                return diff * diff;
            }
        };

        template<typename T>
        struct neg_op<T, 0> { static void Do( T const*, T* ) {} };

        template<typename T, IT N>
        struct cmp_op
        {
            template<typename F>
            static bool Do( const T *lhs, const T *rhs, F const& f )
            {
                if( f(*lhs, *rhs) )
                    return false;

                return cmp_op<T, N-1>::Do(++lhs, ++rhs, f);
            }
            template<typename F>
            static bool Do( const T *a, F const& f )
            {
                if( f(*a) )
                    return false;

                return cmp_op<T, N-1>::Do(++a, f);
            }
        };

        template<typename T>
        struct cmp_op<T, 0>
        {
            template<typename F>
            static bool Do( T const*, T const*, F const& )
            { return true; }
            template<typename F>
            static bool Do( T const*, F const& )
            { return true; } 
        };

        template<typename T, IT N>
        struct accum_op
        {
            template<typename F>
            static T Do( T const *lhs, T const *rhs, F const& f )
            {
                return f(*lhs, *rhs) + accum_op<T, N-1>::Do(lhs + 1, rhs + 1, f);
            }
        };

        template<typename T>
        struct accum_op<T, 0>
        {
            template<typename F>
            static T Do( T const*, T const*, F const& )
            { return 0; }
        };
        

        template<typename T, IT N>
        struct bin_op
        {
            template<typename F>
            static void Do( T *dest, const T *lhs, const T *rhs, F const& f )
            {
                *dest = f(*lhs, *rhs);
                bin_op<T, N-1>::Do(++dest, ++lhs, ++rhs, f);
            }
            template<typename F>
            static void Do( T *dest, const T *lhs, T const value, F const& f )
            {
                *dest = f(*lhs, value);
                bin_op<T, N-1>::Do(++dest, ++lhs, value, f);
            }

        };

        template<typename T>
        struct bin_op<T, 0>
        {
            template<typename F>
            static void Do( T*, T const*, T const*, F const& ) {}
            template<typename F>
            static void Do( T*, T const*, T const, F const& ) {}
        };

        template<typename T>
        class not_near
        {
            T const accuracy;
        public:
            not_near( T const _accuracy ) : accuracy(_accuracy) {}
            bool operator () ( T const lhs, T const rhs ) const
            {
                return !is_near(lhs, rhs, accuracy);
            }
        };

        template<typename T>
        class lerp_hlp
        {
        public:
            lerp_hlp( T const t ) : t_(t) {}
            T operator () ( T const a, T const b ) const
            {
                return lerp(a, b, t_);
            }
            T get_time() const { return t_; }
        private:
            T const t_;
        };

        // max performance initialization policy
        template<typename T>
        struct do_nothing_init_policy
        {
            enum { is_nothing = true };

            template<IT N>
            static void construct( T* )
            {}
        };

        template<typename T, IT N>
        struct for_each_set
        {
            static void Do( T *a, T const v )
            {
                *a = v;
                for_each_set<T, N - 1>::Do( ++a, v );
            }
        };

        template<typename T>
        struct for_each_set<T, 0>
        {
            static void Do( T*, T const )
            {}
        };

        // error safe initialization policy
        template<typename T>
        struct zero_init_policy
        {
            enum { is_nothing = false };

            template<IT N>
            static void construct( T *a )
            {
                for_each_set<T, N>::Do(a, cZero<T>());
            }
        };
    } // namespace nInternal
} // namespace nMath

#endif // GAME_MATH_VECTOR_IMPL_HPP
