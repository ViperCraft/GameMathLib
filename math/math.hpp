
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

#ifndef GAME_MATH_MATH_HPP
#define GAME_MATH_MATH_HPP

#include <cmath>
#include <algorithm>
#include <stdexcept>    // for std::out_of_range
#include <functional>
#include <typeinfo>     // for typeid
#include <cfloat>       // for _finite, _isnan
#include <cassert>
#include <type_traits>

#include "fwd.hpp"
#include "base.hpp"
#include "vector_impl.hpp"
#if defined(__ARM_NEON__)
#   include "neon.hpp"
#endif

namespace nMath
{
    // initialization policy class shall not have members!
    template
    <
        typename              T,
        IT                    ElementCount,
        template<class> class IPolicy
    >
    class Vector : private IPolicy<T>
    {
    public:
        enum
        {
            N = ElementCount
        };

    public:
        typedef Vector<T, N, IPolicy>   VectorType;
        typedef T                       value_type;
        typedef T*                      iterator;
        typedef T const*                const_iterator;
    public:
        Vector()
        {
            IPolicy<T>::template construct<N>( a_ );
        }

        // copy constructor & assigment operator are fine
        // (hold objects by value)

        template<IT M>
        Vector( Vector<T, M> const& rhs )
        {
            enum
            {
                copy_sz = meta::smin<M, N>::result,
                can_copy = (IPolicy<T>::is_nothing == false || copy_sz >= N)
            };
            static_assert( can_copy == true, "cannot copy" );
            std::copy(rhs.a_, rhs.a_ + copy_sz, a_);
            IPolicy<T>::template construct<N - copy_sz>( a_ + copy_sz );
        }

        template<typename U, IT M>
        Vector( Vector<U, M> const& rhs )
        {
            enum
            {
                copy_sz = meta::smin<M, N>::result,
                can_copy = (IPolicy<T>::is_nothing == false || copy_sz >= N)
            };
            static_assert( can_copy == true, "cannot copy" );
            nInternal::cast_op<T, U, copy_sz>::Do( a_, rhs.a_ );
            IPolicy<T>::template construct<N - copy_sz>( a_ + copy_sz );
        }

        explicit Vector( T const val )
        {
            nInternal::for_each_set<T, N>::Do(a_, val);
        }

        template<typename U>
        Vector
        (
            U const x,
            U const y,
            typename std::enable_if<(N == 2), U>::type * = 0
        )
        {
            a_[0] = static_cast<T>(x);
            a_[1] = static_cast<T>(y);
        }

        template<typename U>
        Vector
        (
            U const x,
            U const y,
            U const z,
            typename std::enable_if<(N == 3), U>::type * = 0
        )
        {
            a_[0] = static_cast<T>(x);
            a_[1] = static_cast<T>(y);
            a_[2] = static_cast<T>(z);
        }

        template<typename U>
        Vector
        (
            U const x,
            U const y,
            U const z,
            U const w,
            typename std::enable_if<(N == 4), U>::type * = 0
        )
        {
            a_[0] = static_cast<T>(x);
            a_[1] = static_cast<T>(y);
            a_[2] = static_cast<T>(z);
            a_[3] = static_cast<T>(w);
        }

        template<IT M>
        explicit Vector( const T (&arg)[M] )
        {
            enum { copy_sz = meta::smin<M, N>::result };
            std::copy(arg, arg + copy_sz, a_);
            IPolicy<T>::template construct<N - copy_sz>( a_ + copy_sz );
        }

        template<IT M>
        Vector& operator = ( const T (&arg)[M] )
        {
            enum { copy_sz = meta::smin<M, N>::result };
            std::copy(arg, arg + copy_sz, a_);
            IPolicy<T>::template construct<N - copy_sz>( a_ + copy_sz );
            return *this;
        }

        template<IT M>
        Vector& operator = ( Vector<T, M> const& rhs )
        {
            return operator = ( rhs.a_ );
        }

        template<typename U, IT M>
        Vector& operator = ( Vector<U, M> const& rhs )
        {
            enum { copy_sz = meta::smin<M, N>::result };
            nInternal::cast_op<T, U, copy_sz>::Do(a_, rhs.a_);
            IPolicy<T>::template construct<N - copy_sz>( a_ + copy_sz );
            return *this;
        }

        Vector& operator = ( T val )
        {
            fill(val);
            return *this;
        }

        T& operator [] ( IT idx )
        {
			assert( idx < N );
            return a_[idx];
        }

        template<IT I>
        T& at_c()
        {
            static_assert( I < N, "index too big" );
            return a_[I];
        }

        T const& operator [] ( IT idx ) const
        {
            return const_cast<VectorType*>(this)->operator [](idx);
        }

        template<IT I>
        T const& at_c() const
        {
            static_assert( I < N, "index too big" );
            return a_[I];
        }

        const_iterator    begin() const   { return &a_[0]; }
        iterator          begin()         { return &a_[0]; }

        const_iterator    end() const     { return &a_[N]; }
        iterator          end()           { return &a_[N]; }

        template<template<class> class IP>
        operator Vector<T, N, IP>& ()
        {
            return reinterpret_cast<Vector<T, N, IP>&>(*this);
        }

        T& x()
        {
            return a_[0];
        }

        T& y()
        {
            static_assert( N > 1, "index too big" );
            return a_[1];
        }

        T& z()
        {
            static_assert( N > 2, "index too big" );
            return a_[2];
        }

        T& w()
        {
            static_assert( N > 3, "index too big" );
            return a_[3];
        }

        T x() const
        {
            return a_[0];
        }

        T y() const
        {
            static_assert( N > 1, "index too big" );
            return a_[1];
        }

        T z() const
        {
            static_assert( N > 2, "index too big" );
            return a_[2];
        }

        T w() const
        {
            static_assert( N > 3, "index too big" );
            return a_[3]; 
        }

        Vector operator - () const
        {
            Vector ret;
            nInternal::neg_op<T, N>::Do(a_, ret.a_);
            return ret;
        }

        bool is_equal( VectorType const& vec, T accuracy ) const
        {
            return nInternal::cmp_op<T, N>::Do
            (
                a_,
                vec.a_,
                nInternal::not_near<T>(accuracy)
            );
        }

        bool operator == ( VectorType const& vec ) const
        {
             return nInternal::cmp_op<T, N>::Do(a_, vec.a_, std::not_equal_to<T>());
        }

        bool operator != ( VectorType const& vec ) const
        {
             return !nInternal::cmp_op<T, N>::Do(a_, vec.a_, std::not_equal_to<T>());
        }

        bool operator < ( VectorType const& vec ) const
        {
            return nInternal::cmp_op<T, N>::Do
            (
                a_,
                vec.a_,
                std::not2(std::less<T>())
            );
        }

        bool operator > ( VectorType const& vec ) const
        {
            return nInternal::cmp_op<T, N>::Do
            (
                a_,
                vec.a_,
                std::not2(std::greater<T>())
            );
        }

        Vector& operator += ( VectorType const& other ) 
        {
            nInternal::bin_op<T, N>::Do
            (
                a_,
                a_,
                other.a_,
                std::plus<T>()
            );
            return *this;
        }

        Vector& operator -= ( VectorType const& other ) 
        {
            nInternal::bin_op<T, N>::Do
            (
                a_,
                a_,
                other.a_,
                std::minus<T>()
            );
            return *this;
        }

        Vector& operator *= ( VectorType const& other ) 
        {
            nInternal::bin_op<T, N>::Do
            (
                a_,
                a_,
                other.a_,
                std::multiplies<T>()
            );
            return *this;
        }

        Vector& operator /= ( VectorType const& other ) 
        {
            nInternal::bin_op<T, N>::Do
            (
                a_,
                a_,
                other.a_,
                std::divides<T>()
            );
            return *this;
        }

        Vector& operator += ( T const value )
        {
            nInternal::bin_op<T, N>::Do
            (
                a_,
                a_,
                value,
                std::plus<T>()
            );
            return *this;
        }

        Vector& operator -= ( T const value )
        {
            nInternal::bin_op<T, N>::Do
            (
                a_,
                a_,
                value,
                std::minus<T>()
            );
            return *this;
        }

        Vector& operator *= ( T const value )
        {
            nInternal::bin_op<T, N>::Do
            (
                a_,
                a_,
                value,
                std::multiplies<T>()
            );
            return *this;
        }

        Vector& operator /= ( T const value )
        {
            nInternal::bin_op<T, N>::Do
            (
                a_,
                a_,
                value,
                std::divides<T>()
            );
            return *this;
        }


        friend Vector operator + ( Vector const& lhs, T const value )
        {
            Vector out;
            nInternal::bin_op<T, N>::Do
            (
                out.a_,
                lhs.a_,
                value,
                std::plus<T>()
            );
            return out;
        }

        friend Vector operator - ( Vector const& lhs, T const value )
        {
            Vector out;
            nInternal::bin_op<T, N>::Do
            (
                out.a_,
                lhs.a_,
                value,
                std::minus<T>()
            );
            return out;
        }

        friend Vector operator * ( Vector const& lhs, T const value )
        {
            Vector out;
            nInternal::bin_op<T, N>::Do
            (
                out.a_,
                lhs.a_,
                value,
                std::multiplies<T>()
            );
            return out;
        }

        friend Vector operator / ( Vector const& lhs, T const value )
        {
            Vector out;
            nInternal::bin_op<T, N>::Do
            (
                out.a_,
                lhs.a_,
                value,
                std::divides<T>()
            );
            return out;
        }

        friend Vector operator + ( Vector const& lhs, Vector const& rhs )
        {
            Vector out;
            nInternal::bin_op<T, N>::Do
            (
                out.a_,
                lhs.a_,
                rhs.a_,
                std::plus<T>()
            );
            return out;
        }

        friend Vector operator - ( Vector const& lhs, Vector const& rhs )
        {
            Vector out;
            nInternal::bin_op<T, N>::Do
            (
                out.a_,
                lhs.a_,
                rhs.a_,
                std::minus<T>()
            );
            return out;
        }

        friend Vector operator * ( Vector const& lhs, Vector const& rhs )
        {
            Vector out;
            nInternal::bin_op<T, N>::Do
            (
                out.a_,
                lhs.a_,
                rhs.a_,
                std::multiplies<T>()
            );
            return out;
        }

        friend Vector operator / ( Vector const& lhs, Vector const& rhs )
        {
            Vector out;
            nInternal::bin_op<T, N>::Do
            (
                out.a_,
                lhs.a_,
                rhs.a_,
                std::divides<T>()
            );
            return out;
        }

        void invert()
        {
            nInternal::neg_op<T, N>::Do(a_, a_);
        }

        T dot( VectorType const& other ) const
        {
            return nInternal::accum_op<T, N>::Do(a_, other.a_, std::multiplies<T>());
        }

        T square_sum() const
        {
            return nInternal::accum_op<T, N>::Do(a_, a_, std::multiplies<T>());
        }

        T square_distance( VectorType const& other ) const
        {
            return nInternal::accum_op<T, N>::Do
            (
                other.a_,
                a_,
                nInternal::sqr_diff<T>()
            );
        }

        T magnitude() const
        {
            return std::sqrt( square_sum() );
        }

        void fill( T const val )
        {
            nInternal::for_each_set<T, N>::Do(a_, val);
        }

        void normalize() 
        {
            T ssum = square_sum();
            if( ssum > cSmallEPSILON<T>() )
                (this->operator /= )(std::sqrt(ssum));
        }

        VectorType cross( VectorType const& other ) const
        {
            VectorType out;
            nInternal::cross_prod<T, N>::Do(out.a_, a_, other.a_);
            return out;
        }

        T cos_angle( Vector const& other ) const 
        {
            T _dotProduct       = dot( other );
            T vectorsMagnitude  = magnitude() * other.magnitude();

            return  vectorsMagnitude == cZero<T>() 
                    ?
                    cZero<T>()
                    :
                    _dotProduct / vectorsMagnitude
                    ;
        }

        T angle_between_vectors( Vector const& other ) const
        {
            return std::acos( cos_angle(other) );
        }

        T distance( Vector const& from ) const
        {
            return std::sqrt
            (
                square_distance(from)
            );
        }

        void set_magnitude( T val )
        {
            normalize();
            this->operator *= (val);
        }
    private:
        typedef T (&ref_arrayT)[N];
        typedef const T (&const_ref_arrayT)[N];
    private:
        // member variables
        T                       a_[N];
    public:
        ref_arrayT get_values()
        {
            return a_;
        }
        const_ref_arrayT get_values() const
        {
            return a_;
        }
    private:
        // friend(s)
        template<typename U>
        friend class            Matrix4;
        template<typename U>
        friend class            Quat;
        template<typename U, IT M, template<class> class IP>
        friend class            Vector;
    };

    template
    <
        typename              T,
        IT                    N,
        template<class> class IP0,
        template<class> class IP1
    >
    T dot
    (
        Vector<T, N, IP0> const& lhs,
        Vector<T, N, IP1> const& rhs
    )
    {
        return lhs.dot(rhs);
    }

    template
    <
        typename              T,
        IT                    N,
        template<class> class IP0,
        template<class> class IP1
    >
    Vector<T, N> cross
    (
        Vector<T, N, IP0> const& lhs,
        Vector<T, N, IP1> const& rhs
    )
    {
        return lhs.cross(rhs);
    }

    template
    <
        typename              T,
        IT                    N,
        template<class> class IP0,
        template<class> class IP1
    >
    T distance
    (
        Vector<T, N, IP0> const& lhs,
        Vector<T, N, IP1> const& rhs
    )
    {
        return lhs.distance(rhs);
    }


    template
    <
        typename              T,
        IT                    N,
        template<class> class IP0,
        template<class> class IP1
    >
    T square_distance
    (
        Vector<T, N, IP0> const& lhs,
        Vector<T, N, IP1> const& rhs
    )
    {
        return lhs.square_distance(rhs);
    }

    template
    <
        typename              T,
        IT                    N,
        template<class> class IP0,
        template<class> class IP1
    >
    T cos_angle
    (
        Vector<T, N, IP0>   const& lhs,
        Vector<T, N, IP1>   const& rhs )
    {
        return lhs.cos_angle(rhs);
    }

    template
    <
        typename              T,
        template<class> class IP
    >
    T const& max3( Vector<T, 3, IP>  const& vert )
    {
        return max3(vert[0], vert[1], vert[2]);
    }

    template
    <
        typename              T,
        template<class> class IP
    >
    T const& min3( Vector<T, 3, IP>  const& vert )
    {
        return min3(vert[0], vert[1], vert[2]);
    }

    template
    <
        typename                          T,
        IT                                N,
        template<class> class             IP0,
        template<class> class             IP1,
        template<class> class             IP2
    >
    void lerp
    (
        Vector<T, N, IP0>&          dest,
        Vector<T, N, IP1> const&    a,
        Vector<T, N, IP2> const&    b,
        T const                     t
    )
    {
        nInternal::bin_op<T, N>::Do
        (
            dest.begin(), 
            a.begin(), 
            b.begin(),
            nInternal::lerp_hlp<T>(t)
        );
    }

    template
    <
        typename              T,
        IT                    N,
        template<class> class IP
    >
    bool is_NaN( Vector<T, N, IP> const& v )
    {
        for( IT i = 0; i < N; ++i )
            if( std::isnan(v[i]) )
                return true;

        return false;
    }

    template
    <
        typename              T,
        IT                    N,
        template<class> class IP
    >
    bool is_finite( Vector<T, N, IP> const& v )
    {
        for( IT i = 0; i < N; ++i )
            if( !std::isfinite(v[i]) )
                return false;

        return true;
    }

    // homogeneous dot product
    // valid only for 4-component and 3-component vectors!
    template
    <
        typename              T,
        template<class> class IP0,
        template<class> class IP1
    >
    T doth( Vector<T, 4, IP0> const& vec4, Vector<T, 3, IP1> const& vec3 )
    {
        return  vec4[0] * vec3[0]
                +
                vec4[1] * vec3[1]
                +
                vec4[2] * vec3[2]
                +
                vec4[3]
                ;
    }

    enum
    {
        a00, a10, a20, a30,
        a01, a11, a21, a31,
        a02, a12, a22, a32,
        a03, a13, a23, a33
    };

    template<typename T>
    class Matrix4
    {
    public:
        typedef Matrix4<T>          Matrix4T;
        typedef Vector<T, 4>        Vector4T;
        typedef Vector<T, 3>        Vector3T;
        typedef Vector<T, 2>        Vector2T;
        typedef Quat<T>             QuatT;
        typedef T                   value_type;
        typedef T*                  iterator;
        typedef T const*            const_iterator;
    public:
        enum
        {
            Rows = 4,
            Cols = 4,
            N    = 16
        };

    public:
        // default constructor
        Matrix4() { identity(); }
        
        // identity translate
        template<typename U>
        Matrix4( Vector<U, 3> const& translate )
        {
            m_[0]  = m_[5]  = m_[10] = m_[15] = cOne<T>();
            m_[1]  = m_[2]  = m_[3]  = m_[4]  =
            m_[6]  = m_[7]  = m_[8]  = m_[9]  =
            m_[11] = cZero<T>();
            
            m_[12] = static_cast<T>(translate.x());
            m_[13] = static_cast<T>(translate.y());
            m_[14] = static_cast<T>(translate.z());
        }
        
        // identity translate
        template<typename U>
        Matrix4( Vector<U, 2> const& translate )
        {
            m_[0]  = m_[5]  = m_[10] = m_[15] = cOne<T>();
            m_[1]  = m_[2]  = m_[3]  = m_[4]  =
            m_[6]  = m_[7]  = m_[8]  = m_[9]  =
            m_[11] = m_[14] = cZero<T>();
            
            m_[12] = static_cast<T>(translate.x());
            m_[13] = static_cast<T>(translate.y());
        }

        Matrix4( QuatT const& );

        Matrix4( QuatT const&, Vector3T const& );

        // do nothing constructor ( for performance reason )
        explicit Matrix4( bool ) {}

        template <typename U>
        explicit Matrix4( Matrix4<U> const& other )
        {
            nInternal::cast_op<T, U, N>::Do(m_, other.get_values());
        }

        Matrix4
        (
            Vector4T const& front,
            Vector4T const& up,
            Vector4T const& right,
            Vector4T const& posit
        )
        {
            vFront()    = front;
            vUp()       = up;
            vRight()    = right;
            vPosit()    = posit;
        }

        Matrix4
        (
            T _a00, T _a10, T _a20, T _a30,
            T _a01, T _a11, T _a21, T _a31,
            T _a02, T _a12, T _a22, T _a32,
            T _a03, T _a13, T _a23, T _a33
        )
        {
            m_[a00] = _a00;
            m_[a10] = _a10;
            m_[a20] = _a20;
            m_[a30] = _a30;

            m_[a01] = _a01;
            m_[a11] = _a11;
            m_[a21] = _a21;
            m_[a31] = _a31;

            m_[a02] = _a02;
            m_[a12] = _a12;
            m_[a22] = _a22;
            m_[a32] = _a32;

            m_[a03] = _a03;
            m_[a13] = _a13;
            m_[a23] = _a23;
            m_[a33] = _a33;
        }

        explicit Matrix4( const T (&a_)[N] )
        {
            std::copy(a_, a_ + N, m_);
        }

        // copy constructor & assigment operator are fine

        const_iterator begin() const { return &m_[0];  }
        iterator       begin()       { return &m_[0];  }

        const_iterator end()   const { return &m_[N]; }
        iterator       end()         { return &m_[N]; }


        const Vector4T& vFront() const
        {
            return reinterpret_cast<const Vector4T&>(m_[a00]);
        }

        const Vector4T& vUp() const
        {
            return reinterpret_cast<const Vector4T&>(m_[a01]);
        }

        const Vector4T& vRight() const
        {
            return reinterpret_cast<const Vector4T&>(m_[a02]);
        }

        const Vector4T& vPosit() const
        {
            return reinterpret_cast<const Vector4T&>(m_[a03]);
        }

        Vector4T& vFront()
        {
            return reinterpret_cast<Vector4T&>(m_[a00]);
        }

        Vector4T& vUp()
        {
            return reinterpret_cast<Vector4T&>(m_[a01]);
        }

        Vector4T& vRight()
        {
            return reinterpret_cast<Vector4T&>(m_[a02]);
        }

        Vector4T& vPosit()
        {
            return reinterpret_cast<Vector4T&>(m_[a03]);
        }

        T operator () ( IT row, IT column ) const
        {
            return m_[column*Rows + row];
        }

        T& operator () ( IT row, IT column )
        {
            return m_[column*Rows + row];
        }

        T operator [] ( IT idx ) const
        {
            return m_[idx];
        }

        T& operator [] ( IT idx )
        {
            return m_[idx];
        }

        void identity() 
        {
            m_[0]  = m_[5]  = m_[10] = m_[15] = cOne<T>();
            m_[1]  = m_[2]  = m_[3]  = m_[4]  =
            m_[6]  = m_[7]  = m_[8]  = m_[9]  =
            m_[11] = m_[12] = m_[13] = m_[14] = cZero<T>();
        }

        void rotateAroundX( T theta )
        {
            T c = std::cos(theta);
            T s = std::sin(theta);
            m_[a00] = cOne<T>();
            m_[a11] = c;
            m_[a12] = -s;
            m_[a21] = s;
            m_[a22] = c;
        }

        void rotateAroundY( T theta )
        {
            T c = std::cos(theta);
            T s = std::sin(theta);
            m_[a00] = c;
            m_[a02] = s;
            m_[a11] = cOne<T>();
            m_[a20] = -s;
            m_[a22] = c;
        }

        void rotateAroundZ( T theta )
        {
            T c = std::cos(theta);
            T s = std::sin(theta);
            m_[a00] = c;
            m_[a01] = -s;
            m_[a10] = s;
            m_[a11] = c;
            m_[a22] = cOne<T>();
        }

        void rotateAroundV( T theta, Vector3T const& v )
        {
            T ct = std::cos(theta);
            T st = std::sin(theta);

            T xx = v.a_[0] * v.a_[0];
            T yy = v.a_[1] * v.a_[1];
            T zz = v.a_[2] * v.a_[2];
            T xy = v.a_[0] * v.a_[1];
            T xz = v.a_[0] * v.a_[2];
            T yz = v.a_[1] * v.a_[2];

            m_[a00] = xx + ct * (cOne<T>() - xx);
            m_[a10] = xy + ct * (-xy) + st * (-v.a_[2]);
            m_[a20] = xz + ct * (-xz) + st * v.a_[1];

            m_[a01] = xy + ct * (-xy) + st * v.a_[2];
            m_[a11] = yy + ct * (cOne<T>() - yy);
            m_[a21] = yz + ct * (-yz) + st * (-v.a_[0]);

            m_[a02] = xz + ct * (-xz) + st * (-v.a_[1]);
            m_[a12] = yz + ct * (-yz) + st * v.a_[0];
            m_[a22] = zz + ct * (cOne<T>() - zz);
        }

        // dir vectors must be normalized!!!
        void rotate_to( Vector3T const& dirInit, Vector3T const& dirTo )
        {
            Vector3T lateralDir( dirInit * dirTo );
            vRight() = lateralDir *= reciprocal(lateralDir.square_sum());
            vFront() = vUp() * vRight();
        }
        // Cramer's rule
        void get_inverse( Matrix4T& res ) const
        {
            Matrix4T src_mat(false);
            get_transpose(src_mat);
            const T(&src)[N](src_mat.m_);
            T(&dst)[N](res.m_);
            T   tmp[12]; // temporary pairs
            // calculate pairs for first 8 elements (cofactors)
            tmp[0]  = src[10] * src[15];
            tmp[1]  = src[11] * src[14];
            tmp[2]  = src[9]  * src[15];
            tmp[3]  = src[11] * src[13];
            tmp[4]  = src[9]  * src[14];
            tmp[5]  = src[10] * src[13];
            tmp[6]  = src[8]  * src[15];
            tmp[7]  = src[11] * src[12];
            tmp[8]  = src[8]  * src[14];
            tmp[9]  = src[10] * src[12];
            tmp[10] = src[8]  * src[13];
            tmp[11] = src[9]  * src[12];
            // calculate first 8 elements (cofactors)
            dst[0] =  tmp[0] * src[5] + tmp[3] * src[6] + tmp[4]  * src[7];
            dst[0] -= tmp[1] * src[5] + tmp[2] * src[6] + tmp[5]  * src[7];
            dst[1] =  tmp[1] * src[4] + tmp[6] * src[6] + tmp[9]  * src[7];
            dst[1] -= tmp[0] * src[4] + tmp[7] * src[6] + tmp[8]  * src[7];
            dst[2] =  tmp[2] * src[4] + tmp[7] * src[5] + tmp[10] * src[7];
            dst[2] -= tmp[3] * src[4] + tmp[6] * src[5] + tmp[11] * src[7];
            dst[3] =  tmp[5] * src[4] + tmp[8] * src[5] + tmp[11] * src[6];
            dst[3] -= tmp[4] * src[4] + tmp[9] * src[5] + tmp[10] * src[6];
            dst[4] =  tmp[1] * src[1] + tmp[2] * src[2] + tmp[5]  * src[3];
            dst[4] -= tmp[0] * src[1] + tmp[3] * src[2] + tmp[4]  * src[3];
            dst[5] =  tmp[0] * src[0] + tmp[7] * src[2] + tmp[8]  * src[3];
            dst[5] -= tmp[1] * src[0] + tmp[6] * src[2] + tmp[9]  * src[3];
            dst[6] =  tmp[3] * src[0] + tmp[6] * src[1] + tmp[11] * src[3];
            dst[6] -= tmp[2] * src[0] + tmp[7] * src[1] + tmp[10] * src[3];
            dst[7] =  tmp[4] * src[0] + tmp[9] * src[1] + tmp[10] * src[2];
            dst[7] -= tmp[5] * src[0] + tmp[8] * src[1] + tmp[11] * src[2];
            // calculate pairs for second 8 elements (cofactors)
            tmp[0]  = src[2] * src[7];
            tmp[1]  = src[3] * src[6];
            tmp[2]  = src[1] * src[7];
            tmp[3]  = src[3] * src[5];
            tmp[4]  = src[1] * src[6];
            tmp[5]  = src[2] * src[5];
            tmp[6]  = src[0] * src[7];
            tmp[7]  = src[3] * src[4];
            tmp[8]  = src[0] * src[6];
            tmp[9]  = src[2] * src[4];
            tmp[10] = src[0] * src[5];
            tmp[11] = src[1] * src[4];
            // calculate second 8 elements (cofactors)
            dst[8]  =  tmp[0]  * src[13]  + tmp[3]  * src[14] + tmp[4]  * src[15];
            dst[8]  -= tmp[1]  * src[13]  + tmp[2]  * src[14] + tmp[5]  * src[15];
            dst[9]  =  tmp[1]  * src[12]  + tmp[6]  * src[14] + tmp[9]  * src[15];
            dst[9]  -= tmp[0]  * src[12]  + tmp[7]  * src[14] + tmp[8]  * src[15];
            dst[10] =  tmp[2]  * src[12]  + tmp[7]  * src[13] + tmp[10] * src[15];
            dst[10] -= tmp[3]  * src[12]  + tmp[6]  * src[13] + tmp[11] * src[15];
            dst[11] =  tmp[5]  * src[12]  + tmp[8]  * src[13] + tmp[11] * src[14];
            dst[11] -= tmp[4]  * src[12]  + tmp[9]  * src[13] + tmp[10] * src[14];
            dst[12] =  tmp[2]  * src[10]  + tmp[5]  * src[11] + tmp[1]  * src[9];
            dst[12] -= tmp[4]  * src[11]  + tmp[0]  * src[9]  + tmp[3]  * src[10];
            dst[13] =  tmp[8]  * src[11]  + tmp[0]  * src[8]  + tmp[7]  * src[10];
            dst[13] -= tmp[6]  * src[10]  + tmp[9]  * src[11] + tmp[1]  * src[8];
            dst[14] =  tmp[6]  * src[9]   + tmp[11] * src[11] + tmp[3]  * src[8];
            dst[14] -= tmp[10] * src[11]  + tmp[2]  * src[8]  + tmp[7]  * src[9];
            dst[15] =  tmp[10] * src[10]  + tmp[4]  * src[8]  + tmp[9]  * src[9];
            dst[15] -= tmp[8]  * src[9]   + tmp[11] * src[10] + tmp[5]  * src[8];
            // calculate matrix inverse
            nInternal::bin_op<T, N>::Do
            (
                dst,
                dst,
                // calculate determinant
                reciprocal(src[0] * dst[0] + src[1] * dst[1] + src[2] * dst[2] + src[3] * dst[3]),
                std::multiplies<T>()
                /*
                src[0] * dst[0] + src[1] * dst[1] + src[2] * dst[2] + src[3] * dst[3],
                std::divides<T>()
                */
            );
        }

        Matrix4T get_inverse() const
        {
            Matrix4T Res(false);
            get_inverse(Res);
            return Res;
        }

        void get_transpose( Matrix4T &r ) const
        {
            r.m_[a00] = m_[a00];
            r.m_[a01] = m_[a10];
            r.m_[a02] = m_[a20];
            r.m_[a03] = m_[a30];
            r.m_[a10] = m_[a01];
            r.m_[a11] = m_[a11];
            r.m_[a12] = m_[a21];
            r.m_[a13] = m_[a31];
            r.m_[a20] = m_[a02];
            r.m_[a21] = m_[a12];
            r.m_[a22] = m_[a22];
            r.m_[a23] = m_[a32];
            r.m_[a30] = m_[a03];
            r.m_[a31] = m_[a13];
            r.m_[a32] = m_[a23];
            r.m_[a33] = m_[a33];
        }

        Matrix4T    get_transpose() const
        {
            Matrix4T Res(false);
            get_transpose(Res);
            return Res;
        }

        void        translate_by( T x, T y, T z )
        {
            m_[a03] += x;
            m_[a13] += y;
            m_[a23] += z;
        }

        void        translate( Vector2T const& vec )
        {
            m_[a03] = vec.a_[0];
            m_[a13] = vec.a_[1];
        }

        void        translate( T x, T y, T z )
        {
            m_[a03] = x;
            m_[a13] = y;
            m_[a23] = z;
        }

        void        translate( Vector3T const& v )
        {
            m_[a03] = v.a_[0];
            m_[a13] = v.a_[1];
            m_[a23] = v.a_[2];
        }

        void        translate( Matrix4<T> const& other )
        {
            m_[a03] = other.m_[a03];
            m_[a13] = other.m_[a13];
            m_[a23] = other.m_[a23];
        }

        void        scale( T all )
        {
            m_[0]  *= all;
            m_[1]  *= all;
            m_[2]  *= all;

            m_[4]  *= all;
            m_[5]  *= all;
            m_[6]  *= all;

            m_[8]  *= all;
            m_[9]  *= all;
            m_[10] *= all;
        }
        
        void        set_scale( T all )
        {
            m_[0]  = all;
            m_[1]  = all;
            m_[2]  = all;
            
            m_[4]  = all;
            m_[5]  = all;
            m_[6]  = all;
            
            m_[8]  = all;
            m_[9]  = all;
            m_[10] = all;
        }

        void        scale( T x, T y, T z )
        {
            m_[0]  *= x;
            m_[1]  *= y;
            m_[2]  *= z;

            m_[4]  *= x;
            m_[5]  *= y;
            m_[6]  *= z;

            m_[8]  *= x;
            m_[9]  *= y;
            m_[10] *= z;
        }

        void        scale( Vector3T const& vec )
        {
            m_[0]  *= vec.a_[0];
            m_[1]  *= vec.a_[1];
            m_[2]  *= vec.a_[2];

            m_[4]  *= vec.a_[0];
            m_[5]  *= vec.a_[1];
            m_[6]  *= vec.a_[2];

            m_[8]  *= vec.a_[0];
            m_[9]  *= vec.a_[1];
            m_[10] *= vec.a_[2];
        }

        // get translation of the current matrix
        void get_pos( Vector3T& vec ) const
        {
            vec.a_[0] = m_[a03];
            vec.a_[1] = m_[a13];
            vec.a_[2] = m_[a23];
        }

        Vector3T get_pos() const
        {
            Vector3T out;
            get_pos(out);
            return out;
        }

        // fill translate values with zero
        void clear_trans() 
        {
            m_[a03] = m_[a13] = m_[a23] = cZero<T>();
        }

        Matrix4T& operator *= ( Matrix4T const& m )
        {
            Matrix4T t(*this);
            mult_impl(m_, t.m_, m.m_);
            return *this;
        }

        friend Matrix4T operator * ( Matrix4T const& t, Matrix4T const& m )
        {
            Matrix4T product(false);
            mult_impl(product.m_, t.m_, m.m_);
            return product;
        }

        Vector4T operator * ( Vector4T const& v ) const 
        {
            return Vector4T
            (
                m_[a00] * v.a_[0] + m_[a01] * v.a_[1] + m_[a02] * v.a_[2] + m_[a03] * v.a_[3],
                m_[a10] * v.a_[0] + m_[a11] * v.a_[1] + m_[a12] * v.a_[2] + m_[a13] * v.a_[3],
                m_[a20] * v.a_[0] + m_[a21] * v.a_[1] + m_[a22] * v.a_[2] + m_[a23] * v.a_[3],
                m_[a30] * v.a_[0] + m_[a31] * v.a_[1] + m_[a32] * v.a_[2] + m_[a33] * v.a_[3]
            );
        }

        Vector3T operator * ( Vector3T const& v ) const 
        {
            return Vector3T
            (
                m_[a00] * v.a_[0] + m_[a01] * v.a_[1] + m_[a02] * v.a_[2] + m_[a03],
                m_[a10] * v.a_[0] + m_[a11] * v.a_[1] + m_[a12] * v.a_[2] + m_[a13],
                m_[a20] * v.a_[0] + m_[a21] * v.a_[1] + m_[a22] * v.a_[2] + m_[a23]
            );
        }
        
        Vector2T operator * ( Vector2T const& v ) const
        {
            return Vector2T
            (
                m_[a00] * v.a_[0] + m_[a01] * v.a_[1] + m_[a03],
                m_[a10] * v.a_[0] + m_[a11] * v.a_[1] + m_[a13]
            );
        }
        

        Vector3T rotate( Vector3T const& v ) const
        {
            return Vector3T
            (
                m_[a00] * v.a_[0] + m_[a01] * v.a_[1] + m_[a02] * v.a_[2],
                m_[a10] * v.a_[0] + m_[a11] * v.a_[1] + m_[a12] * v.a_[2],
                m_[a20] * v.a_[0] + m_[a21] * v.a_[1] + m_[a22] * v.a_[2]
            );
        }

        Vector3T un_rotate( Vector3T const& v ) const
        {
            return Vector3T
            (
                m_[a00] * v.a_[0] + m_[a10] * v.a_[1] + m_[a20] * v.a_[2],
                m_[a01] * v.a_[0] + m_[a11] * v.a_[1] + m_[a21] * v.a_[2],
                m_[a02] * v.a_[0] + m_[a12] * v.a_[1] + m_[a22] * v.a_[2]
            );
        }

        // both vectors must be normalized!
        void rotation_from( Vector3T const& u, Vector3T const& v )
        {
            T    phi, h, lambda;

            Vector3T w(u.cross(v));

            phi       = u.dot(v);
            lambda    = w.dot(w);

            if( lambda > cSmallEPSILON<T>() )
                h = (cOne<T>() - phi) / lambda;
            else
                h = lambda;

            T hxy = w[0] * w[1] * h;
            T hxz = w[0] * w[2] * h;
            T hyz = w[1] * w[2] * h;

            m_[a00] = phi + w[0] * w[0] * h;
            m_[a01] = hxy - w[2];
            m_[a02] = hxz + w[1];

            m_[a10] = hxy + w[2];
            m_[a11] = phi + w[1] * w[1] * h;
            m_[a12] = hyz - w[0];

            m_[a20] = hxz - w[1];
            m_[a21] = hyz + w[0];
            m_[a22] = phi + w[2] * w[2] * h;
        }

        Matrix4T& look_at( Vector3T const& eye, Vector3T const& center, Vector3T const& up )
        {
            Vector3T x, y, z( eye - center );

            z.normalize();

            // X vector = Y cross Z
            x = cross(up, z);

            // Y vector = Z cross X
            y = cross(z, x);

            x.normalize();
            y.normalize();

            m_[a00] = x.x();       m_[a01] = x.y();       m_[a02] = x.z();
            m_[a03] = -x.x() * eye.x() - x.y() * eye.y() - x.z() * eye.z();
            m_[a10] = y.x();       m_[a11] = y.y();       m_[a12] = y.z();
            m_[a13] = -y.x() * eye.x() - y.y() * eye.y() - y.z() * eye.z();
            m_[a20] = z.x();       m_[a21] = z.y();       m_[a22] = z.z();
            m_[a23] = -z.x() * eye.x() - z.y() * eye.y() - z.z() * eye.z();
            m_[a30] = cZero<T>();  m_[a31] = cZero<T>();  m_[a32] = cZero<T>();  m_[a33] = cOne<T>();
            return *this;
        }

        void frustum( T l, T r, T b, T t, T n, T f ) 
        {
            m_[a00] = (cTwo<T>() * n) / (r - l);
            m_[a10] = cZero<T>();
            m_[a20] = cZero<T>();
            m_[a30] = cZero<T>();

            m_[a01] = cZero<T>();
            m_[a11] = (cTwo<T>() * n) / (t - b);
            m_[a21] = cZero<T>();
            m_[a31] = cZero<T>();

            m_[a02] = (r + l) / (r - l);
            m_[a12] = (t + b) / (t - b);
            m_[a32] = -cOne<T>();

            m_[a03] = cZero<T>();
            m_[a13] = cZero<T>();
            m_[a33] = cZero<T>();
            
            // support far plane infinity
            // see http://http.developer.nvidia.com/GPUGems/gpugems_ch09.html
            if( f != T(INFINITY) )
            {
                m_[a22] = -(f + n) / (f - n);
                m_[a23] = -(cTwo<T>() * f * n) / (f - n);
            }
            else
            {
                m_[a22] = -cOne<T>();
                m_[a23] = -cTwo<T>() * n;
            }
        }

        // fovy in degress
        void perspective( T fovy, T aspect, T n, T f ) 
        {
            T xmin, xmax, ymin, ymax;

            ymax = n * std::tan(fovy * cHalf<T>());
            ymin = -ymax;

            xmin = ymin * aspect;
            xmax = ymax * aspect;

            frustum(xmin, xmax, ymin, ymax, n, f);
        }

        void ortho
        (
            T left,
            T right,
            T bottom,
            T top,
            T n,
            T f
        ) 
        {
            m_[a00] = cTwo<T>() / (right - left);
            m_[a01] = cZero<T>();
            m_[a02] = cZero<T>();
            m_[a03] = - (right + left) / (right - left);
            m_[a10] = cZero<T>();
            m_[a11] = cTwo<T>() / (top - bottom);
            m_[a12] = cZero<T>();
            m_[a13] = - (top + bottom) / (top - bottom);
            m_[a20] = cZero<T>();
            m_[a21] = cZero<T>();
            m_[a22] = -cTwo<T>() / (f - n);
            m_[a23] = -(f + n) / (f - n);
            m_[a30] = cZero<T>();
            m_[a31] = cZero<T>();
            m_[a32] = cZero<T>();
            m_[a33] = cOne<T>();            
        }

        static Matrix4<T> makePitchMatrix   ( T angle );
        static Matrix4<T> makeYawMatrix     ( T angle );
        static Matrix4<T> makeRollMatrix    ( T angle );
        static Matrix4<T> makeFromDir       ( Vector3T const& dir );
    private:
        static void mult_impl( T (&d)[N], const T (&a)[N], const T (&b)[N] )
        {
            d[0]  = a[0] * b[0] + a[4] * b[1] + a[8]  * b[2] + a[12] * b[3];
            d[1]  = a[1] * b[0] + a[5] * b[1] + a[9]  * b[2] + a[13] * b[3];
            d[2]  = a[2] * b[0] + a[6] * b[1] + a[10] * b[2] + a[14] * b[3];
            d[3]  = a[3] * b[0] + a[7] * b[1] + a[11] * b[2] + a[15] * b[3];
            d[4]  = a[0] * b[4] + a[4] * b[5] + a[8]  * b[6] + a[12] * b[7];
            d[5]  = a[1] * b[4] + a[5] * b[5] + a[9]  * b[6] + a[13] * b[7];
            d[6]  = a[2] * b[4] + a[6] * b[5] + a[10] * b[6] + a[14] * b[7];
            d[7]  = a[3] * b[4] + a[7] * b[5] + a[11] * b[6] + a[15] * b[7];
            d[8]  = a[0] * b[8] + a[4] * b[9] + a[8]  * b[10] + a[12] * b[11];
            d[9]  = a[1] * b[8] + a[5] * b[9] + a[9]  * b[10] + a[13] * b[11];
            d[10] = a[2] * b[8] + a[6] * b[9] + a[10] * b[10] + a[14] * b[11];
            d[11] = a[3] * b[8] + a[7] * b[9] + a[11] * b[10] + a[15] * b[11];
            d[12] = a[0] * b[12] + a[4] * b[13] + a[8]  * b[14] + a[12] * b[15];
            d[13] = a[1] * b[12] + a[5] * b[13] + a[9]  * b[14] + a[13] * b[15];
            d[14] = a[2] * b[12] + a[6] * b[13] + a[10] * b[14] + a[14] * b[15];
            d[15] = a[3] * b[12] + a[7] * b[13] + a[11] * b[14] + a[15] * b[15];
        }
    private:
        typedef T (&ref_arrayT)[N];
        typedef const T (&const_ref_arrayT)[N];
    public:
        ref_arrayT get_values()
        {
            return m_;
        }
        const_ref_arrayT get_values() const
        {
            return m_;
        }
    private:
        T m_[N];
    private:
        friend class Quat<T>;
    };

    template<typename T>
    Matrix4<T> Matrix4<T>::makePitchMatrix( T angle )
    {
        T cos_ang = std::cos(angle);
        T sin_ang = std::sin(angle);

        return Matrix4<T>
        (
            cOne<T>(),     cZero<T>(),  cZero<T>(),  cZero<T>(),
            cZero<T>(),    cos_ang,     sin_ang,     cZero<T>(),
            cZero<T>(),    -sin_ang,    cos_ang,     cZero<T>(),
            cZero<T>(),    cZero<T>(),  cZero<T>(),  cOne<T>()
        );
    }

    template<typename T>
    Matrix4<T> Matrix4<T>::makeYawMatrix( T angle )
    {
        T cos_ang = std::cos(angle);
        T sin_ang = std::sin(angle);

        return Matrix4<T>
        (
            cos_ang,       cZero<T>(),     -sin_ang,    cZero<T>(),
            cZero<T>(),    cOne<T>(),      cZero<T>(),  cZero<T>(),
            sin_ang,       cZero<T>(),     cos_ang,     cZero<T>(),
            cZero<T>(),    cZero<T>(),     cZero<T>(),  cOne<T>()
        );
    }

    template<typename T>
    Matrix4<T> Matrix4<T>::makeRollMatrix( T angle )
    {
        T cos_ang = std::cos(angle);
        T sin_ang = std::sin(angle);

        return Matrix4<T>
        (
            cos_ang,       sin_ang,     cZero<T>(), cZero<T>(),
            -sin_ang,      cos_ang,     cZero<T>(), cZero<T>(),
            cZero<T>(),    cZero<T>(),  cOne<T>(),  cZero<T>(),
            cZero<T>(),    cZero<T>(),  cZero<T>(), cOne<T>()
        );
    }

    // using the GramSchidth procedure
    template<typename T>
    Matrix4<T> Matrix4<T>::makeFromDir( typename Matrix4::Vector3T const& dir )
    {        
        Vector3T up, right, front(dir); 

	    front *= cOne<T>() / std::sqrt( dot(front, front) );
	    if (abs (front.z()) > static_cast<T>(0.577) ) {
		    right = cross(front, Vector3T (-front.y(), front.z(), cZero<T>()));
	    } else {
	  	    right = cross(front, Vector3T(-front.y(), front.z(), cZero<T>()));
	    }
  	    right *= cOne<T>() / std::sqrt( dot(right, right) );
  	    up = cross(right, front);

	    return Matrix4<T>
        (
            Vector4T (front.x(), front.y(), front.z(), cZero<T>()), 
            Vector4T (up.x(), up.y(), up.z(), cZero<T>()), 
            Vector4T (right.x(), right.y(), right.z(), cZero<T>()), 
            Vector4T (cZero<T>(), cZero<T>(), cZero<T>(), cOne<T>())
        );
    }

    enum
    {
        aX, aY, aZ, aW
    };

    template<typename T>
    class Quat
    {
    public:
        typedef Quat<T>         QuatT;
        typedef Vector<T, 3>    Vector3T;
        typedef Matrix4<T>      Matrix4T;
        typedef T               value_type;
        typedef T*              iterator;
        typedef T const*        const_iterator;
    public:
        enum
        {
            Rows = 1,
            Cols = 4,
            N    = 4
        };

    public:
        // default constructor
        Quat()
        { identity(); }

        // copy constructor & assigment operator are fine

        template<template<class> class IP>
        Quat( Vector<T, N, IP> const& v )
        {
            std::copy(v.a_, v.a_ + N, q_);
        }

        Quat( Matrix4T const& mat )
        {
            from_matrix(mat);
        }

        explicit Quat
        (
            T const& x,
            T const& y,
            T const& z,
            T const& w
        )
        {
            q_[aX] = x;
            q_[aY] = y;
            q_[aZ] = z;
            q_[aW] = w;
        }

        Quat( T angle, Vector3T const& axis )
        {
            T len = axis.square_sum();

            if( len > cSmallEPSILON<T>() )
            {
                T   half_angle = angle * cHalf<T>(),
                    scale      = std::sin(half_angle) / std::sqrt(len)
                    ;

                q_[aX]   = scale * axis[0];
                q_[aY]   = scale * axis[1];
                q_[aZ]   = scale * axis[2];
                q_[aW]   = std::cos(half_angle);
            }
            else
            {
                identity();
            }
        }

        explicit Quat( const T (&arg)[N] )
        {
            std::copy(arg, arg + N, q_);
        }

        const_iterator    begin() const   { return &q_[0]; }
        iterator          begin()         { return &q_[0]; }

        const_iterator    end() const     { return &q_[N]; }
        iterator          end()           { return &q_[N]; }

        T & operator [] ( IT idx )
        {
			assert( idx < N );
            return q_[idx];
        }

        const T & operator [] ( IT idx ) const
        {
			assert( idx < N );
            return q_[idx];
        }

        T& x()
        { return q_[aX]; }
        T& y()
        { return q_[aY]; }
        T& z()
        { return q_[aZ]; }
        T& w()
        { return q_[aW]; }

        T const& x() const
        { return q_[aX]; }
        T const& y() const
        { return q_[aY]; }
        T const& z() const
        { return q_[aZ]; }
        T const& w() const
        { return q_[aW]; }


        bool is_equal( QuatT const& other, T accuracy ) const
        {
            return nInternal::cmp_op<T, N>::Do
            (
                q_,
                other.q_,
                nInternal::not_near<T>(accuracy)
            );
        }

        // WARNING! quaternion must be normalized!
        Vector3T to_euler_angles() const
        {
            Vector3T ret;
            T test = q_[aX] * q_[aY] + q_[aZ] * q_[aW];
            if( test > cNearHalf<T>() )
            {
                ret[0] = cTwo<T>() * std::atan2(q_[aX], q_[aW]);
                ret[1] = cPI<T>() / cTwo<T>();
                ret[2] = cZero<T>();
            }
            else 
            if( test < -cNearHalf<T>() )
            {
                ret[0] = -cTwo<T>() * std::atan2(q_[aX], q_[aW]);
                ret[1] = -cPI<T>() / cTwo<T>();
                ret[2] = cZero<T>();
            }
            else
            {
                T   sx = q_[aX] * q_[aX],
                    sy = q_[aY] * q_[aY],
                    sz = q_[aZ] * q_[aZ];

                ret[0] = std::atan2
                (
                    cTwo<T>() * q_[aY] * q_[aW] - cTwo<T>() * q_[aX] * q_[aZ],
                    cOne<T>() - cTwo<T>() * sy - cTwo<T>() * sz
                );

                ret[1] = std::asin(cTwo<T>() * test);
                ret[2] = std::atan2
                (
                    cTwo<T>() * q_[aX]    * q_[aW] - cTwo<T>() * q_[aY] * q_[aZ],
                    cOne<T>() - cTwo<T>() * sx     - cTwo<T>() * sz
                );
            }

            return ret;
        }

        void rotation_arc( Vector3T const& from, Vector3T const& to )
        {
            Vector3T c( from.cross(to) );
            T d    = from.dot(to);
            q_[aX] = c[0];
            q_[aY] = c[1];
            q_[aZ] = c[2];
            q_[aW] = d + std::sqrt( from.square_sum() * to.square_sum() );
        }

        void rotation_arc2( Vector3T const& from, Vector3T const& to )
        {
            Vector3T c( from.cross(to) );
            q_[aX] = c.a_[0];
            q_[aY] = c.a_[1];
            q_[aZ] = c.a_[2];
            q_[aW] = from.dot(to);
            normalize();
            q_[aW] += cOne<T>();
            if( q_[aW] <= cSmallEPSILON<T>() )
            {
                if( from.a_[2] * from.a_[2] > from.a_[0] * from.a_[0] )
                {
                    q_[aX] = cZero<T>();
                    q_[aY] = from.a_[2];
                    q_[aZ] = -from.a_[1];
                }
                else
                {
                    q_[aX] = from.a_[1];
                    q_[aY] = -from.a_[0];
                    q_[aZ] = cZero<T>();
                }
            }
            normalize();
        }

        void to_axis_angle( Vector3T& axis, T& angle )
        {
            T vl = std::sqrt( q_[aX] * q_[aX] + q_[aY] * q_[aY] + q_[aZ] * q_[aZ] );
            if( vl > cSmallEPSILON<T>() )
            {
                axis *= reciprocal(vl);
                if( q_[aW] < 0 )
                    angle = cTwo<T>() * std::atan2(-vl, -q_[aW]);
                else
                    angle = cTwo<T>() * std::atan2( vl,  q_[aW]);
            }
            else
            {
                axis    = cZero<T>();
                angle   = cZero<T>();
            }
        }

        T norm() const
        {
            return q_[aX] * q_[aX] + q_[aY] * q_[aY] + q_[aZ] * q_[aZ] + q_[aW] * q_[aW];
        }

        T magnitude() const
        {
            return std::sqrt(norm());
        }

        void conjugate()
        {
            q_[aX] = -q_[aX];
            q_[aY] = -q_[aY];
            q_[aZ] = -q_[aZ];
        }

        QuatT& normalize()
        {
            T m = magnitude();
            if( m < cSmallEPSILON<T>() )
            {
                stabilize_length();
                m = magnitude();
            }

            *this *= reciprocal(m);

            return *this;
        }

        void identity()
        {
            q_[aX] = cZero<T>();
            q_[aY] = cZero<T>();
            q_[aZ] = cZero<T>();
            q_[aW] = cOne<T>();
        }

        void stabilize_length()
        {
            T cs = std::abs(q_[aX]) + std::abs(q_[aY]) + std::abs(q_[aZ]) + std::abs(q_[aW]);
            if( cs > cZero<T>() )
                *this *= reciprocal(cs);
            else
                identity();
        }

        QuatT operator -() const
        {
            return QuatT(-q_[aX], -q_[aY], -q_[aZ], q_[aW]);
        }

        void invert()
        {
            q_[aX] = -q_[aX];
            q_[aY] = -q_[aY];
            q_[aZ] = -q_[aZ];
        }

        T dot( QuatT const& other ) const
        {
            return q_[aX] * other.q_[aX] + q_[aY] * other.q_[aY] + q_[aZ] * other.q_[aZ] + q_[aW] * other.q_[aW];
        }

        // method by Norel Mikhail
        void shortest_arc( Vector3T const& from, Vector3T const& to )
        {
            Vector3T c( from.cross(to) );
            q_[aX] = c.a_[0];
            q_[aY] = c.a_[1];
            q_[aZ] = c.a_[2];
            q_[aW] = from.dot(to);

            normalize();
            q_[aW] += cOne<T>();

            if( q_[aW] <= cSmallEPSILON<T>() )
            {
                if( (from.a_[2] * from.a_[2]) > (from.a_[0] * from.a_[0]) )
                {
                    q_[aX] = cZero<T>();
                    q_[aY] = from.a_[2];
                    q_[aZ] = -from.a_[1];
                }
                else
                {
                    q_[aX] = from.a_[1];
                    q_[aY] = -from.a_[0];
                    q_[aZ] = cZero<T>();
                }
            }
            normalize();
        }

        void slerp( QuatT const& a, QuatT const& b, T t )
        {
            T cosine = a.dot(b);
            if( cosine < -cOne<T>() )
                cosine = -cOne<T>();
            else if( cosine > cOne<T>() )
                cosine = cOne<T>();

            T angle = std::acos(cosine);

            if( std::abs(angle) < cSmallEPSILON<T>() )
            {
                *this = a;
            }
            else
            {
                T sine     = std::sin(angle);
                T sine_inv = reciprocal(sine);
                T c1       = std::sin((cOne<T>() - t) * angle) * sine_inv;
                T c2       = std::sin(t * angle) * sine_inv;
                q_[aX]     = c1 * a.q_[aX] + c2 * b.q_[aX];
                q_[aY]     = c1 * a.q_[aY] + c2 * b.q_[aY];
                q_[aZ]     = c1 * a.q_[aZ] + c2 * b.q_[aZ];
                q_[aW]     = c1 * a.q_[aW] + c2 * b.q_[aW];
            }
        }

        // this method correctly handle 360 degree turn
        void slerp2( QuatT const& a, QuatT const& _b, T t )
        {
            QuatT b(_b);

            T cosine = a.dot(b);
            if( cosine < -cOne<T>() )
                cosine = -cOne<T>();
            else if( cosine > cOne<T>() )
                cosine = cOne<T>();

            if( cosine < cZero<T>() )
            {
                cosine  = -cosine;
                b.invert();
            }

            T angle = std::acos(cosine);

            if( std::abs(angle) < cSmallEPSILON<T>() )
            {
                *this = a;
            } 
            else 
            {
                T sine      = std::sin(angle);
                T sine_inv  = reciprocal(sine);
                T c1        = std::sin((cOne<T>() - t) * angle) * sine_inv;
                T c2        = std::sin(t * angle) * sine_inv;
                q_[aX]      = c1 * a.q_[aX] + c2 * b.q_[aX];
                q_[aY]      = c1 * a.q_[aY] + c2 * b.q_[aY];
                q_[aZ]      = c1 * a.q_[aZ] + c2 * b.q_[aZ];
                q_[aW]      = c1 * a.q_[aW] + c2 * b.q_[aW];
            }
        }
        // WARNING! fill only 3x3 matrix!
        void to_matrix( Matrix4T &mat, T s ) const
        {
            const T x2 = q_[aX] * s;
            const T y2 = q_[aY] * s;
            const T z2 = q_[aZ] * s;

            const T xx = x2 * q_[aX];
            const T xy = x2 * q_[aY];
            const T xz = x2 * q_[aZ];
            const T xw = x2 * q_[aW];

            const T yy = y2 * q_[aY];
            const T yz = y2 * q_[aZ];
            const T yw = y2 * q_[aW];

            const T zz = z2 * q_[aZ];
            const T zw = z2 * q_[aW];

            mat.m_[a00] = cOne<T>() - (yy + zz);
            mat.m_[a10] =             (xy - zw);
            mat.m_[a20] =             (xz + yw);

            mat.m_[a01] =             (xy + zw);
            mat.m_[a11] = cOne<T>() - (xx + zz);
            mat.m_[a21] =             (yz - xw);

            mat.m_[a02] =             (xz - yw);
            mat.m_[a12] =             (yz + xw);
            mat.m_[a22] = cOne<T>() - (xx + yy);
        }

        void to_matrix( Matrix4T &mat ) const
        {
            to_matrix(mat, cTwo<T>());
        }

        // quaternion can be non-normalized
        // WARNING! fill only 3x3 matrix!
        void to_matrix2( Matrix4T &mat ) const
        {
            const T n = norm();
            const T s = n > cZero<T>() ? cTwo<T>() / n : cZero<T>();
            to_matrix(mat, s);
        }

        void from_matrix( Matrix4T const& mat )
        {
            T const trace = mat.m_[a00] + mat.m_[a11] + mat.m_[a22];

            if (trace > cZero<T>())
            {
                T scale = std::sqrt(trace + cOne<T>());
                q_[aW]  = cHalf<T>() * scale;
                scale   = cHalf<T>() / scale;
                q_[aX]  = (mat.m_[a12] - mat.m_[a21]) * scale;
                q_[aY]  = (mat.m_[a20] - mat.m_[a02]) * scale;
                q_[aZ]  = (mat.m_[a01] - mat.m_[a10]) * scale;
            }
            else
            {
                static IT const next[] = { 1, 2, 0 };

                IT  i = 0;
                if (mat.m_[a11] > mat.m_[a00])
                    i = 1;
                if (mat.m_[a22] > mat(i, i))
                    i = 2;

                IT const j = next[i], k = next[j];

                T scale = std::sqrt(cOne<T>() + mat(i, i) - mat(j, j) - mat(k, k));
                q_[i]  = cHalf<T>() * scale;
                // get inv for division
                scale  = cHalf<T>() / scale;
                q_[j]  = (mat(j, i) + mat(i, j)) * scale;
                q_[k]  = (mat(k, i) + mat(i, k)) * scale;
                q_[aW] = (mat(j, k) - mat(k, j)) * scale;
            }
        }

        bool operator == ( QuatT const& other ) const
        {
             return 
                 q_[aX] == other.q_[aX]
                 &&
                 q_[aY] == other.q_[aY]
                 &&
                 q_[aZ] == other.q_[aZ]
                 &&
                 q_[aW] == other.q_[aW]
                 ;
        }

        QuatT& operator *= ( QuatT const& q )
        {
            QuatT p(*this);
            q_[aX] = p.q_[aW] * q.q_[aX] + p.q_[aX] * q.q_[aW] + p.q_[aY] * q.q_[aZ] - p.q_[aZ] * q.q_[aY],
            q_[aY] = p.q_[aW] * q.q_[aY] + p.q_[aY] * q.q_[aW] + p.q_[aZ] * q.q_[aX] - p.q_[aX] * q.q_[aZ],
            q_[aZ] = p.q_[aW] * q.q_[aZ] + p.q_[aZ] * q.q_[aW] + p.q_[aX] * q.q_[aY] - p.q_[aY] * q.q_[aX],
            q_[aW] = p.q_[aW] * q.q_[aW] - p.q_[aX] * q.q_[aX] - p.q_[aY] * q.q_[aY] - p.q_[aZ] * q.q_[aZ];
            return *this;
        }


        QuatT& operator *= ( T const& value )
        {
            q_[aX] *= value,
            q_[aY] *= value,
            q_[aZ] *= value,
            q_[aW] *= value;
            return *this;
        }

        QuatT operator * ( T const& value )
        {
            QuatT p(*this);
            p *= value;
            return p;
        }

        friend QuatT operator + ( QuatT const& lhs, QuatT const& rhs )
        {
            return QuatT
            (
                lhs.q_[aX] + rhs.q_[aX],
                lhs.q_[aY] + rhs.q_[aY],
                lhs.q_[aZ] + rhs.q_[aZ],
                lhs.q_[aW] + rhs.q_[aW]
            );
        }

        friend QuatT operator - ( QuatT const& lhs, QuatT const& rhs )
        {
            return QuatT
            (
                lhs.q_[aX] - rhs.q_[aX],
                lhs.q_[aY] - rhs.q_[aY],
                lhs.q_[aZ] - rhs.q_[aZ],
                lhs.q_[aW] - rhs.q_[aW]
            );
        }

        friend QuatT operator * ( QuatT const& lhs, QuatT const& rhs )
        {
            return QuatT
            (
                lhs.q_[aW] * rhs.q_[aX] + lhs.q_[aX] * rhs.q_[aW] + lhs.q_[aY] * rhs.q_[aZ] - lhs.q_[aZ] * rhs.q_[aY],
                lhs.q_[aW] * rhs.q_[aY] + lhs.q_[aY] * rhs.q_[aW] + lhs.q_[aZ] * rhs.q_[aX] - lhs.q_[aX] * rhs.q_[aZ],
                lhs.q_[aW] * rhs.q_[aZ] + lhs.q_[aZ] * rhs.q_[aW] + lhs.q_[aX] * rhs.q_[aY] - lhs.q_[aY] * rhs.q_[aX],
                lhs.q_[aW] * rhs.q_[aW] - lhs.q_[aX] * rhs.q_[aX] - lhs.q_[aY] * rhs.q_[aY] - lhs.q_[aZ] * rhs.q_[aZ]
            );
        }
    private:
        typedef T (&ref_arrayT)[N];
        typedef const T (&const_ref_arrayT)[N];
    private:
        // member variables
        T               q_[N];
    public:
        ref_arrayT get_values()
        {
            return q_;
        }
        const_ref_arrayT get_values() const
        {
            return q_;
        }
    };

    template<typename T>
    Quat<T> lerp( Quat<T> const& a, Quat<T> const& b, T t )
    {
        return Quat<T>( a + ( b - a ) * t );
    }

    template<typename T>
    Quat<T> lerp2( Quat<T> const& a, Quat<T> const& b, T t )
    {
        if( a.Dot(b) < cZero<T>() )
        {
            return Quat<T>( a + ( -b - a ) * t );
        } else
        {
            return Quat<T>( a + ( b - a ) * t );
        }
    }

    template<typename T>
    Quat<T> conjugate(const Quat<T> &q)
    {
        Quat<T> t = q;
        t.conjugate();
        return t;
    }

    template<typename T>
    Matrix4<T>::Matrix4
    (
        Quat<T>      const& quat,
        Vector<T, 3> const& pos
    )
    {
        quat.to_matrix2(*this);
        m_[a03] = pos.a_[0];
        m_[a13] = pos.a_[1];
        m_[a23] = pos.a_[2];
        m_[a30] = cZero<T>();
        m_[a31] = cZero<T>();
        m_[a32] = cZero<T>();
        m_[a33] = cOne<T>();
    }

    template<typename T>
    Matrix4<T>::Matrix4
    (
        Quat<T> const& quat
    )
    {
        quat.to_matrix2(*this);

        m_[a30] = m_[a31] = m_[a32] = m_[a03] = m_[a13] = m_[a23] = cZero<T>();
        m_[a33] = cOne<T>();
    }

    template<typename VecType>
    class Plane
    {
    public:
        typedef typename VecType::value_type    value_type;
    public:
        // default constructor
        Plane() 
            : 
            n_(cZero<value_type>()),
            d_(cZero<value_type>())
            {}

        // copy constructor & assigment operator are fine
        Plane( VecType const& normal, value_type d )
            :
            n_(normal),
            d_(d)
            {
                normalize();
            }

        // WARNING! normal must normalized
        Plane( VecType const& normal, VecType const& point )
            :
            n_(normal),
            d_(point.dot(normal))
            {}
        
        void init( VecType const& normal, VecType const& point )
        {
            n_ = normal;
            d_ = point.dot(normal);
        }

        // return distance from point to plane
        value_type get_distance( VecType const& point ) const
        {
            return n_.dot(point) - d_;
        }

        // return true if ray(start, dir) intersect plane and fraction value
        bool ray_intersection
        (
            VecType const& start,
            VecType const& dir,
            value_type   & fraction
        )
        const
        {
            value_type      fDdN        = dir.dot(n_),
                            fSDistance  = get_distance(start),
                            abs_fDdN    = std::abs(fDdN)
                            ;

            if( abs_fDdN > cSmallEPSILON<value_type>() )
            {
                fraction = -fSDistance / fDdN;
                return true;
            }

            if( abs_fDdN <= cSmallEPSILON<value_type>() )
            {
                fraction = cZero<value_type>();
                return true;
            }

            return false;
        }
        VecType const& n() const   { return n_;  }
        value_type              d() const   { return d_;  }
    private:
        void normalize()
        {
            value_type t = n_.square_sum();

            if( t != cOne<value_type>() )
            {
                t   = std::sqrt(t);
                n_  /= t;
                d_  /= t;
            }
        }
    private:
        VecType       n_;
        value_type    d_;
    };
} // namespace nMath

#endif // !GAME_MATH_MATH_HPP
