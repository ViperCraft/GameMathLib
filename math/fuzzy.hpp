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

#ifndef GAME_MATH_FUZZY_HPP
#define GAME_MATH_FUZZY_HPP

#include "math.hpp"
#include <cassert>

namespace nMath
{
    namespace nInternal
    {
        template<typename T>
        struct DefaultFuzzyChecking;
    } // namespace nInternal

    template
    <
        typename T,
        template<typename> class CheckingPolicy = nInternal::DefaultFuzzyChecking
    >
    class fuzzyType
        :
        private CheckingPolicy<T>
    {
    public:
        using CheckingPolicy<T>::CheckValue;
    public:
        fuzzyType()
            :
            value_(cZero<T>())
            {}

        explicit fuzzyType( T v )
            :
            value_(v)
            {
                CheckValue(v);
            }

        fuzzyType& normalize()
        {
            if( value_ >= cHalf<T>() )
                value_ = cOne<T>();
            else
                value_ = cZero<T>();
            return *this;
        }

        fuzzyType& operator = ( T v )
        {
            CheckValue(value_);
            CheckValue(v);

            value_ = v;

            return *this;
        }

        fuzzyType& operator |= ( fuzzyType const& v )
        {
            CheckValue(value_);
            CheckValue(v.value_);

            value_ = (std::max)(value_, v.value_);

            return *this;
        }

        fuzzyType& operator &= ( fuzzyType const& v )
        {
            CheckValue(value_);
            CheckValue(v.value_);

            value_ = (std::min)(value_, v.value_);

            return *this;
        }

        operator T   () const        { return value_; }

        T get() const                { return value_; }

        fuzzyType operator !()
        {
            return fuzzyType(cOne<T>() - value_);
        }
    private:
        T        value_;
    };

    template<typename T>
    T very( fuzzyType<T> const& f )
    {
        T value = f.get();
        return value * value;
    }

    namespace nInternal
    {
        template<typename T>
        struct DefaultFuzzyChecking
        {
            static void CheckValue( const T& value )
            {
                assert( value >= cZero<T>() && value <= cOne<T>() );
            }
        };
    } // namespace nInternal

    template<typename T>
    fuzzyType<T> operator || ( fuzzyType<T> const& lhs, fuzzyType<T> const& rhs )
    {
        fuzzyType<T> ret(lhs);
        ret |= rhs;
        return ret;
    }

    template<typename T>
    fuzzyType<T> operator && ( fuzzyType<T> const& lhs, fuzzyType<T> const& rhs )
    {
        fuzzyType<T> ret(lhs);
        ret &= rhs;
        return ret;
    }

    typedef fuzzyType<float>    fuzzyf;
    typedef fuzzyType<double>   fuzzyd;

    template<typename T>
    bool f_false( fuzzyType<T> const& v )
    {
        return v == cZero<T>();
    }

    template<typename T>
    bool f_true( fuzzyType<T> const& v )
    {
        return v == cOne<T>();
    }

    namespace nProfiles
    {
        template<typename T>
        fuzzyType<T> spike
        (
            T    value,
            T    lo,
            T    high
        )
        {
            typedef fuzzyType<T> fuzzyT;
            T peak;

            value -= lo;

            if( (lo < cZero<T>()) && (high < cZero<T>()) )
            {
                high = -(high - lo);
            }
            else if( (lo < cZero<T>()) && (high > cZero<T>()) )
            {
                high += -lo;
            }
            else if( (lo > cZero<T>()) && (high > cZero<T>()) )
            {
                high -= lo;
            }

            peak = (high * cHalf<T>());

            if( value < peak )
            {
                return fuzzyT( value / peak );
            }
            else if( value > peak )
            {
                return fuzzyT( high - value / peak );
            }

            return fuzzyT(cOne<T>());
        }

        template<typename T>
        fuzzyType<T> trim
        (
            T   value,
            T   lo,
            T   med,
            T   hi
        )
        {
            typedef fuzzyType<T> fuzzyT;

            if( value <= lo )
                return fuzzyT(cZero<T>());
            if( lo <= value && value <= med )
                return fuzzyT( (value - lo) / (med - lo) );
            if( med <= value && value <= hi )
                return fuzzyT( (hi - value) / (hi - med) );

            return fuzzyT(cZero<T>());
        }

        template<typename T>
        fuzzyType<T> plateau
        (
            T    value,
            T    lo,
            T    lo_plat,
            T    hi_plat,
            T    hi
        )
        {
            typedef fuzzyType<T> fuzzyT;

            value += -lo;

            if( lo < cZero<T>() )
            {
                lo_plat += -lo;  hi_plat += -lo;
                hi      += -lo;  lo       = cZero<T>();
            } else
            {
                lo_plat -= lo;  hi_plat -= lo;
                hi      -= lo;  lo       = cZero<T>();
            }

            if( value < lo )
                return fuzzyT(cZero<T>());
            else if( value > hi )
                return fuzzyT(cZero<T>());
            else if( (value >= lo_plat) && (value <= hi_plat) )
                return fuzzyT(cOne<T>());
            else if( value < lo_plat )
            {
                T dLo = lo_plat - lo;
                if( dLo != cZero<T>() )
                    return fuzzyT((value - lo) * reciprocal(dLo));
                else
                    return fuzzyT(value - lo);
            }
            else if( value > hi_plat )
            {
                T dHi = hi - hi_plat;
                if( dHi != cZero<T>() )
                    return fuzzyT((hi - value) * reciprocal(dHi));
                else
                    return fuzzyT(hi - value);
            }

            return fuzzyT(cZero<T>());
        }

        template<typename T>
        fuzzyType<T> plateau
        (
            fuzzyType<T> const& value,
            T                   lo,
            T                   lo_plat,
            T                   hi_plat,
            T                   hi
        )
        {
            return plateau<T>(value.get(), lo, lo_plat, hi_plat, hi);
        }
    } // namespace nProfiles
} // namespace nMath

#endif // GAME_MATH_FUZZY_HPP
