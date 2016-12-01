
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

#ifndef GAME_MATH_UTILS_HPP
#define GAME_MATH_UTILS_HPP

#include "math.hpp"

namespace nMath
{
    template<typename T>
    T const& min3( T const& a, T const& b, T const& c )
    {
        return ((a < b) && (a < c)) ? a : ((b < c) ? b : c);
    }

    template<typename T>
    T const& max3( T const& a, T const& b, T const& c )
    {
        return ((a > b) && (a > c)) ? a : ((b > c) ? b : c);
    }

    template<typename T> IT bitcount( T value )
    {
        IT bitCount = 0;

        for( int i = 0; i < sizeof(T); ++i )
        {
            T byte = value & 0xFF;

            bitCount += (byte & 0x01) ? 1 : 0;
            bitCount += (byte & 0x02) ? 1 : 0;
            bitCount += (byte & 0x04) ? 1 : 0;
            bitCount += (byte & 0x08) ? 1 : 0;
            bitCount += (byte & 0x10) ? 1 : 0;
            bitCount += (byte & 0x20) ? 1 : 0;
            bitCount += (byte & 0x40) ? 1 : 0;
            bitCount += (byte & 0x80) ? 1 : 0;

            value = value >> 8;
        }
        return bitCount;
    }

    template<typename T> bool is_pow2( T value )
    {
        if( value == 1 )
            return true;

        if
        (
            value < 1
            ||
            value & ( value - 1 )
        )
            return false;

        return true;
    }

    template<typename T>  T next_pow2( T v )
    {
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        return ++v;
    }

    // return 1 with sign of given value
    template<typename T>
    T sign( T const& v )
    {
        if( v >= cZero<T>() )
            return cOne<T>();

        return -cOne<T>();
    }

    // return false if negative sign
    template<typename T>
    T sign_bool( T const& v )
    {
        if( v >= cZero<T>() )
            return true;

        return false;
    }

    // Hermite interpolation
    template<typename VType, typename TType>
    void herp
    (
        VType           &dest,
        VType const&    p0,
        VType const&    p1,
        VType const&    p2,
        VType const&    p3,
        TType const&    t
    )
    {
        typedef typename VType::value_type T;

        T t2  = t * t;
        T t3  = t2 * t;
        T kp0 = (cTwo<T>()   * t2 -               t3 - t         ) * cHalf<T>();
        T kp1 = (cThree<T>() * t3 - cFive<T>()  * t2 + cTwo<T>() ) * cHalf<T>();
        T kp2 = (cFour<T>()  * t2 - cThree<T>() * t3 + t         ) * cHalf<T>();
        T kp3 = (t3 - t2)    * cHalf<T>();

        dest  = p0 * kp0 + p1 * kp1 + p2 * kp2 + p3 * kp3;
    }
    
    

    inline IT log2( IT x )
    {
        x |= (x >> 1);
        x |= (x >> 2);
        x |= (x >> 4);
        x |= (x >> 8);
        x |= (x >> 16);
        x >>= 1;
        x -= (x >> 1)  & 0x55555555U;
        x  = ((x >> 2) & 0x33333333U) + (x & 0x33333333U);
        x  = ((x >> 4) + x) & 0x0F0F0F0FU;
        x += x >> 8;
        x += x >> 16;
        return x & 63;
    }

    template
    <
        typename T,
        template<class> class IP
    >
    T project_on_Z( Matrix4<T> const& m, Vector<T, 3, IP> const& v )
    {
        typename Matrix4<T>::const_ref_arrayT _m(m.get_values());
        return _m[a20] * v.a_[0] + _m[a21] * v.a_[1] + _m[a22] * v.a_[2] + _m[a23];
    }

    template
    <
        typename T,
        template<class> class IP
    >
    Vector<T, 4, IP> pre_mult_plane( Matrix4<T> const& m, Vector<T, 4, IP> const& plane )
    {
        return Vector<T, 4, IP>
        (
            m.vFront().dot( plane ),
            m.vUp()   .dot( plane ),
            m.vRight().dot( plane ),
            m.vPosit().dot( plane )
        );
    }

    template<typename T, typename U>
    bool is_equal( T const& lhs, T const& rhs, const U& accuracy )
    {
        for
        (
            typename T::const_iterator beg0 = lhs.begin(), beg1 = rhs.begin(),
            end = lhs.end();
            beg0 != end;
            ++beg0, ++beg1
        )
        {
            if( is_near(*beg0, *beg1, accuracy) == false )
                return false;
        }
        return true;
    }

    template
    <
        typename                T,
        IT                      N,
        template<class> class   IP
    >
    bool get_line2plane_intersec_point
    (
        Vector<T, N, IP> const&  lineStart,
        Vector<T, N, IP> const&  lineEnd,
        Vector<T, N, IP> const&  pointOnPlane,
        Vector<T, N, IP> const&  planeNormal,
        Vector<T, N, IP>&        intersection,
        T&                       tOut
    )
    {
        Vector<T, N, IP>    vDirection(lineEnd - lineStart);
        T                   fLineLength(vDirection.dot(planeNormal));

        if( std::abs(fLineLength) < cSmallEPSILON<T>() )
            return false;

        Vector<T, N, IP>    L1(pointOnPlane - lineStart);
        T                   fDistanceFromPlane(L1.dot(planeNormal));

        tOut              = fDistanceFromPlane / fLineLength;

        // correct tOut
        //*
        if
        (
            is_near
            (
                tOut,
                cZero<T>(),
                cSmallEPSILON<T>()
            )
        )
            tOut = cZero<T>();

        if
        (
            is_near
            (
                tOut, 
                cOne<T>(), 
                cSmallEPSILON<T>()
            )
        )
            tOut = cOne<T>();
        //*/

        if
        (
            tOut < cZero<T>()
            ||
            tOut > cOne<T>()
        )
            return false;

        intersection = lineStart + vDirection * tOut;

        return true;
    }

    template
    <
        typename                T,
        IT                      N,
        template<class> class   IP
    >
    bool get_line2plane_intersec_point
    (
        Vector<T, N, IP> const&  lineStart,
        Vector<T, N, IP> const&  lineEnd,
        Vector<T, N, IP> const&  pointOnPlane,
        Vector<T, N, IP> const&  planeNormal,
        T&                       tOut
    )
    {
        Vector<T, N, IP>    vDirection(lineEnd - lineStart);
        T                   fLineLength(vDirection.dot(planeNormal));

        if( std::abs(fLineLength) < cSmallEPSILON<T>() )
            return false;

        Vector<T, N, IP>    L1(pointOnPlane - lineStart);
        T                   fDistanceFromPlane(L1.dot(planeNormal));

        tOut              = fDistanceFromPlane / fLineLength;

        // correct tOut
        //*
        if
        (
            is_near
            (
                tOut,
                cZero<T>(),
                cSmallEPSILON<T>()
            )
        )
            tOut = cZero<T>();

        if
        (
            is_near
            (
                tOut,
                cOne<T>(),
                cSmallEPSILON<T>()
            )
        )
            tOut = cOne<T>();
        //*/

        if
        (
            tOut < cZero<T>()
            ||
            tOut > cOne<T>()
        )
            return false;

        return true;
    }

    template
    <
        typename                T,
        IT                      N,
        template<class> class   IP
    >
    bool get_line2plane_intersec_point
    (
        Vector<T, N, IP> const&  lineStart,
        Vector<T, N, IP> const&  lineEnd,
        Vector<T, N, IP> const&  pointOnPlane,
        Vector<T, N, IP> const&  planeNormal
    )
    {
        Vector<T, N, IP>    vDirection(lineEnd - lineStart);
        T                   fLineLength(vDirection.dot(planeNormal));

        if( std::abs(fLineLength) < cSmallEPSILON<T>() )
            return false;

        Vector<T, N, IP>    L1(pointOnPlane - lineStart);
        T                   fDistanceFromPlane(L1.dot(planeNormal));

        T tOut            = fDistanceFromPlane / fLineLength;

        // correct tOut
        //*
        if
        (
            is_near
            (
                tOut,
                cZero<T>(),
                cSmallEPSILON<T>()
            )
        )
            tOut = cZero<T>();

        if
        (
            is_near
            (
                tOut,
                cOne<T>(),
                cSmallEPSILON<T>()
            )
        )
            tOut = cOne<T>();
        //*/

        if
        (
            tOut < cZero<T>()
            ||
            tOut > cOne<T>()
        )
            return false;

        return true;
    }

    template
    <
        // must exsist accessor(s)
        // Vector<T, 2, IP> const& get_v( const PolyContT&, size_t );
        // and typename PointContT::const_iterator
        typename PointContT,
        typename T,
        template<class> class   IP
    >
    bool point_in_poly
    (
        const PointContT&        poly,
        Vector<T, 2, IP> const&  pt
    )
    {
        bool pass = false;
        typename PointContT::const_iterator iter = get_begin(poly),
                                            end = get_end(poly),
                                            last_iter = end
                                            ;
        --last_iter;
        for( ;iter != end; last_iter = iter++ )
        {
            const Vector<T, 2> &pi( get_vec(iter) ),
                               &pj( get_vec(last_iter) )
                               ;
            if
            (
                ((pi[1] <= pt[1]) && (pt[1] < pj[1]))
                ||
                ((pj[1] <= pt[1]) && (pt[1] < pi[1]))
            )
            {
                if( pt[0] - pi[0] < ((pj[0] - pi[0]) * (pt[1] - pi[1]) / (pj[1] - pi[1])) )
                {
                    pass = !pass;
                }
            }
        }
        return pass;
    }

    template
    <
        typename T,
        template<class> class   IP
    >
    bool point_in_circle
    (
        Vector<T, 2, IP> const& point,
        Vector<T, 2, IP> const& center,
        T const                 radius
    )
    {
        return square_distance(point, center) <= (radius * radius);
    }

    namespace intersection {
        
    template<typename VecT>
    bool circle_circle
    ( 
        VecT const& cen0, typename VecT::value_type r0,
        VecT const& cen1, typename VecT::value_type r1
    )
    {
        typename VecT::value_type ssr = r0 + r1;
        return (cen0 - cen1).square_sum() < ssr * ssr;
    }

    template
    <
        typename T,
        template<class> class   IP
    >
    bool line_circle
    (
        Vector<T, 2, IP> const& from,
        Vector<T, 2, IP> const& to,
        Vector<T, 2, IP> const& center,
        T const&                radius
    )
    {
        Vector<T, 2, IP> a0(from - center),
                         a1(to   - center)
                         ;
        return
            (
                (radius * radius)
                *
                square_distance(a0, a1)
                -
                sqr( a0.x() * a1.y() - a1.x() * a0.y() )
            )
            >=
            cZero<T>()
            ;
    }

    template
    <
        typename T,
        template<class> class   IP
    >
    bool ray_circle
    (
        Vector<T, 2, IP> const& org,
        Vector<T, 2, IP> const& dir,
        Vector<T, 2, IP> const& center,
        T const&                radius
    )
    {
        Vector<T, 2, IP> dV(org - center);
        T c = dV.square_sum() - radius * radius;
        if( c <= cZero<T>() )
            return true;
        T b = dV.Dot(dir);
        if( b >= cZero<T>() )
            return false;
        return (b * b) >= c;
    }

    template<typename T>
    void closest_point_on_line_from_point
    (
        T const& x1, T const& y1,
        T const& x2, T const& y2,
        T const& px, T const& py,
        T& nx,       T& ny
    )
    {
       T vx = x2 - x1;
       T vy = y2 - y1;
       T wx = px - x1;
       T wy = py - y1;

       T c1 = vx * wx + vy * wy;
       T c2 = vx * vx + vy * vy;

       T ratio = c1 / c2;

       nx = x1 + ratio * vx;
       ny = y1 + ratio * vy;
    }
        
    template<typename T>
    inline void closest_point_on_segment_from_point(const T& x1, const T& y1,
                                                    const T& x2, const T& y2,
                                                    const T& px, const T& py,
                                                    T& nx,       T& ny)
    {
        T vx = x2 - x1;
        T vy = y2 - y1;
        T wx = px - x1;
        T wy = py - y1;
        
        T c1 = vx * wx + vy * wy;
        
        if (c1 <= T(0.0))
        {
            nx = x1;
            ny = y1;
            return;
        }
        
        T c2 = vx * vx + vy * vy;
        
        if (c2 <= c1)
        {
            nx = x2;
            ny = y2;
            return;
        }
        
        T ratio = c1 / c2;
        
        nx = x1 + ratio * vx;
        ny = y1 + ratio * vy;
    }

        
    template
    <
        typename              T,
        IT                    N,
        template<class> class IP
    >
    Vector<T, N, IP> closest_point_on_segment_from_point
    (
        Vector<T, N, IP> const& s0,
        Vector<T, N, IP> const& s1,
        Vector<T, N, IP> const& p
    )
    {
        Vector<T, N, IP> const   v1( s1 - s0 ),
                                 v2( p - s0 )
                                 ;
        
        T const c1 = dot(v1, v2);
        
        if( c1 <= cZero<T>() )
            return s0;
        
        T const c2 = dot(v1, v1);
        
        if( c2 <= c1 )
            return s1;
        
        T const ratio = c1 / c2;
        
        return p + v1 * ratio;
    }

    template
    <
        typename T,
        template<class> class IP
    >
    Vector<T, 2, IP> closest_point_on_line_from_point
    (
        Vector<T, 2, IP> const& a0,
        Vector<T, 2, IP> const& a1,
        Vector<T, 2, IP> const& p
    )
    {
        Vector<T, 2, IP> out;
        closest_point_on_line_from_point
        (
            a0.x(),     a0.y(),
            a1.x(),     a1.y(),
            p.x(),      p.y(),
            out.x(),    out.y()
        );
        return out;
    }
        
    template
    <
        typename T,
        template<class> class IP
    >
    inline bool segment_circle
    (
        Vector<T, 2, IP> const& s0,
        Vector<T, 2, IP> const& s1,
        Vector<T, 2, IP> const& c0,
        float                   radius
    )
    {
        Vector<T, 2, IP> out;
        //( closest_point_on_segment_from_point(s0, s1, c0) );
        closest_point_on_segment_from_point(s0.x(), s0.y(), s1.x(), s1.y(), c0.x(), c0.y(), out.x(), out.y());
        return (square_distance(out, c0) <= (radius * radius));
    }

    template
    <
        typename T,
        template<class> class IP
    >
    bool segment_line
    (
        Vector<T, 2, IP> const& a0,
        Vector<T, 2, IP> const& a1,
        Vector<T, 2, IP> const& b0,
        Vector<T, 2, IP> const& b1,
        Vector<T, 2, IP>&       x
    )
    {
        return segment_segment
        (
            a0,
            a1,
            closest_point_on_line_from_point(b0, b1, a0),
            closest_point_on_line_from_point(b0, b1, a1),
            x
        );
    }
        
    template<typename T>
    bool is_on_segment(T xi, T yi, T xj, T yj, T xk, T yk)
    {
        return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) &&
        (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
    }
    
    template<typename T>
    int compute_direction(T xi, T yi, T xj, T yj, T xk, T yk)
    {
        T a = (xk - xi) * (yj - yi);
        T b = (xj - xi) * (yk - yi);
        return a < b ? -1 : a > b ? 1 : 0;
    }
        
    template<typename T>
    bool segment_segment
    (
        T x1,  T y1,
        T x2,  T y2,
        T x3,  T y3,
        T x4,  T y4
    )
    {
        int d1 = compute_direction(x3, y3, x4, y4, x1, y1);
        int d2 = compute_direction(x3, y3, x4, y4, x2, y2);
        int d3 = compute_direction(x1, y1, x2, y2, x3, y3);
        int d4 = compute_direction(x1, y1, x2, y2, x4, y4);
        return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
                ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) ||
        (d1 == 0 && is_on_segment(x3, y3, x4, y4, x1, y1)) ||
        (d2 == 0 && is_on_segment(x3, y3, x4, y4, x2, y2)) ||
        (d3 == 0 && is_on_segment(x1, y1, x2, y2, x3, y3)) ||
        (d4 == 0 && is_on_segment(x1, y1, x2, y2, x4, y4));
    }


    template<typename T>
    bool segment_segment
    (
        T x1,  T y1,
        T x2,  T y2,
        T x3,  T y3,
        T x4,  T y4,
        T &ix, T &iy
    )
    {
        T ax = x2 - x1;
        T bx = x3 - x4;

        T lowerx;
        T upperx;
        T uppery;
        T lowery;

        if( ax < cZero<T>() )
        {
           lowerx = x2;
           upperx = x1;
        }
        else
        {
           upperx = x2;
           lowerx = x1;
        }

        if( bx > cZero<T>() )
        {
           if( (upperx < x4) || (x3 < lowerx) )
           return false;
        }
        else 
        if( (upperx < x3) || (x4 < lowerx) )
           return false;

        T ay = y2 - y1;
        T by = y3 - y4;

        if( ay < cZero<T>() )
        {
           lowery = y2;
           uppery = y1;
        }
        else
        {
           uppery = y2;
           lowery = y1;
        }

        if( by > cZero<T>() )
        {
           if( (uppery < y4) || (y3 < lowery) )
              return false;
        }
        else 
        if( (uppery < y3) || (y4 < lowery) )
           return false;

        T cx = x1 - x3;
        T cy = y1 - y3;
        T d  = (by * cx) - (bx * cy);
        T f  = (ay * bx) - (ax * by);

        if( f > cZero<T>() )
        {
           if( (d < cZero<T>()) || (d > f) )
              return false;
        }
        else 
        if( (d > cZero<T>()) || (d < f) )
           return false;

        T e = (ax * cy) - (ay * cx);

        if( f > cZero<T>() )
        {
           if( (e < cZero<T>()) || (e > f) )
              return false;
        }
        else
        if( (e > cZero<T>()) || (e < f) )
           return false;

        T ratio = (ax * -by) - (ay * -bx);

        if( is_near(ratio, cZero<T>()) )
        {
           ratio = ((cy * -bx) - (cx * -by)) / ratio;
           ix    = x1 + (ratio * ax);
           iy    = y1 + (ratio * ay);
        }
        else
        {
           if( is_near((ax * -cy),(-cx * ay)) )
           {
              ix = x3;
              iy = y3;
           }
           else
           {
              ix = x4;
              iy = y4;
           }
        }

        return true;
    }
        
    template
    <
        typename T,
        template<class> class IP
    >
    bool segment_segment
    (
        Vector<T, 2, IP> const& a0,
        Vector<T, 2, IP> const& a1,
        Vector<T, 2, IP> const& b0,
        Vector<T, 2, IP> const& b1
    )
    {
        return segment_segment
        (
            a0.x(), a0.y(),
            a1.x(), a1.y(),
            b0.x(), b0.y(),
            b1.x(), b1.y()
        );
    }

    template
    <
        typename T,
        template<class> class IP
    >
    bool segment_segment
    (
        Vector<T, 2, IP> const& a0,
        Vector<T, 2, IP> const& a1,
        Vector<T, 2, IP> const& b0,
        Vector<T, 2, IP> const& b1,
        Vector<T, 2, IP>&       x
    )
    {
        return segment_segment
        (
            a0.x(), a0.y(),
            a1.x(), a1.y(),
            b0.x(), b0.y(),
            b1.x(), b1.y(),
            x.x(),  x.y()
        );
    }

    template
    <
        typename              T,
        template<class> class IP
    >
    bool line_line
    (
        Vector<T, 2, IP> const& p1,
        Vector<T, 2, IP> const& p2,
        Vector<T, 2, IP> const& c1,
        Vector<T, 2, IP> const& c2,
        Vector<T, 2, IP>&       q
    )
    {
        T d = (p2.x() - p1.x()) * (c1.y() - c2.y()) - (c1.x() - c2.x()) * (p2.y() - p1.y());
        if( d == cZero<T>() )
            return false;

        T   Dp = (c1.x() - p1.x()) * (c1.y() - c2.y()) - (c1.x() - c2.x()) * (c1.y() - p1.y()),
            Dc = (p2.x() - p1.x()) * (c1.y() - p1.y()) - (c1.x() - p1.x()) * (p2.y() - p1.y())
            ;

        T   tp = Dp / d,
            tc = Dc / d
            ;

        if
        (
            (tc >= cZero<T>() && tc <= cOne<T>())
            &&
            (tp >= cZero<T>() && tp <= cOne<T>())
        )
        {

            q.x() = p1.x() + tp * (p2.x() - p1.x());
            q.y() = p1.y() + tp * (p2.y() - p1.y());

            return true;
        }

        return false;
    }

    } // namespace intersection

    template
    <
        typename              T,
        template<class> class IP
    >
    Vector<T, 3, IP> get_polar3
    (
        Vector<T, 3, IP> const& a
    )
    {
        T R             = a.magnitude();
        T latitude      = std::asin(a[1] / R);
        T azimuth       = std::atan2(a[2], a[0]);
        
        return Vector<T, 3, IP>(R, azimuth, latitude);
    }
    
    template
    <
        typename              T,
        template<class> class IP
    >
    Vector<T, 2, IP> get_polar2( Vector<T, 2, IP> const& a )
    {
        T R  = a.magnitude();
        
        return Vector<T, 2, IP>(R, std::atan2(a.y(), a.x()));
    }

    template<typename Vec>
    typename Vec::value_type get_full_angle
    (
        Vec const&                             a,
        Vec const&                             b,
        Vec const&                             a_norm,
        typename Vec::value_type const&        accuracy = cSmallEPSILON<typename Vec::value_type>()
    )
    {
        typedef typename Vec::value_type T;
        T c_angle = cos_angle(a, b);

        if( is_near(c_angle, cOne<T>(), accuracy) )
            c_angle = cOne<T>();

        T angle = std::acos(c_angle);

        if( _isnan(angle) )
            angle = cZero<T>();

        if( c_angle == cOne<T>() )
            angle = cZero<T>();
        else if( is_near(c_angle, -cOne<T>(), accuracy) )
            angle = cPI<T>();

        if( a_norm.dot(b) < cZero<T>() )
            angle = cTwo<T>() * cPI<T>() - angle;

        return angle;
    }

    template<typename Vec>
    typename Vec::value_type point_from_line_distance
    (
        const Vec& test_point,
        const Vec& point_on_line,
        const Vec& line,
        const Vec& line_norm
    )
    {
        return
        (
            std::abs
            (
                line_norm.dot(test_point - point_on_line)
            )
            /
            line.magnitude()
        );
    }

    template
    <
        typename              T,
        template<class> class IP
    >
    void normalize_plane( Vector<T, 4, IP>& vPlane )
    {
        T mag = reciprocalSQRT
        (
            vPlane[0] * vPlane[0]
            +
            vPlane[1] * vPlane[1]
            +
            vPlane[2] * vPlane[2]
        );

        vPlane *= mag;
    }

    template
    <
        typename              T,
        IT                    N,
        template<class> class IP
    >
    T nearest_distance_from_segment
    (
        Vector<T, N, IP> const& p,
        Vector<T, N, IP> const& seg_p0,
        Vector<T, N, IP> const& seg_p1,
        T (*d) ( Vector<T, N, IP> const&, Vector<T, N, IP> const& ) = &nMath::distance<T, N, IP, IP>
    )
    {
        Vector<T, N, IP>    v = seg_p1 - seg_p0,
                            w = p - seg_p0
                            ;
        T c1 = dot(w, v);

        if( c1 <= cZero<T>() )
            return d(p, seg_p0);
        T c2 = dot(v, v);
        if( c2 <= c1 )
            return d(p, seg_p1);

        T b = c1 / c2;
        Vector<T, N, IP> pb = seg_p0 + v * b;
        return d(p, pb);
    }

    template
    <
        typename              T,
        IT                    N,
        template<class> class IP
    >
    bool nearest_distance_from_segment2
    (
        Vector<T, N, IP> const& p,
        Vector<T, N, IP> const& seg_p0,
        Vector<T, N, IP> const& seg_p1,
        T                     & result,
        T (*d) ( Vector<T, N, IP> const&, Vector<T, N, IP> const& ) = &nMath::distance<T, N, IP, IP>
    )
    {
        Vector<T, N, IP>    v           = seg_p1 - seg_p0;
        T                   line_mag    = d(seg_p1, seg_p0);
        if( line_mag <= cSmallEPSILON<T>() )
            return false;
        T u =
            ((p.x() - seg_p0.x()) * (seg_p1.x() - seg_p0.x())) +
            ((p.y() - seg_p0.y()) * (seg_p1.y() - seg_p0.y())) +
            ((p.z() - seg_p0.z()) * (seg_p1.z() - seg_p0.z()))
            ;
        u /= line_mag * line_mag;

        if( u < cZero<T>() || u > cOne<T>() )
            return false;

        Vector<T, N, IP> intersection = seg_p0 + v * u;
        result = d(p, intersection);
        return true;
    }
    
    template<typename T>
    inline void closest_point_on_rectangle_from_point(const T x1, const T y1,
                                                      const T x2, const T y2,
                                                      const T px, const T py,
                                                      T& nx,       T& ny)
    {
        if (px < std::min(x1,x2))
            nx = std::min(x1,x2);
        else if (px > std::max(x1,x2))
            nx = std::max(x1,x2);
        else
            nx = px;
        
        if (py < std::min(y1,y2))
            ny = std::min(y1,y2);
        else if (py > std::max(y1,y2))
            ny = std::max(y1,y2);
        else
            ny = py;
    }
    
    template
    <
        typename              T,
        template<class> class IP
    >
    inline Vector<T, 2, IP> closest_point_on_rectangle_from_point
    (
        Vector<T, 2, IP> const& v0,
        Vector<T, 2, IP> const& v1,
        Vector<T, 2, IP> const& p0
    )
    {
        Vector<T, 2, IP> res;
        
        closest_point_on_rectangle_from_point(v0.x(), v0.y(), v1.x(), v1.y(), p0.x(), p0.y(), res.x(), res.y());
        
        return res;
    }
    
    
    
    template
    <
        typename              T,
        template<class> class IP
    >
    bool rectangle_circle
    (
        Vector<T, 2, IP> const& v0, Vector<T, 2, IP> const& v1/*rectangle*/
        , Vector<T, 2, IP> const& p0, T const radius/*circle*/
    )
    {
        return  point_in_circle(closest_point_on_rectangle_from_point(v0, v1, p0), p0, radius);
    }

    template
    <
        typename              T,
        IT                    N,
        template<class> class IP
    >
    bool aabb_overlap_test
    (
        Vector<T, N, IP> const& box0_min,
        Vector<T, N, IP> const& box0_max,
        Vector<T, N, IP> const& box1_min,
        Vector<T, N, IP> const& box1_max
    )
    {
        return (box0_min < box1_max) && (box0_max > box1_min);
    }

} // namespace nMath

#endif // BOOST_MATH_UTILS_HPP
