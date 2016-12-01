#pragma once

#include "math.hpp"

#if !defined(__ARM_NEON__)
# error __ARM_NEON__ must be defined!!!
#endif

#include <arm_neon.h>


namespace nMath {
    
    namespace nInternal
    {
        template<>
        struct bin_op<float, 2>
        {
            /*
            template<typename F>
            static void Do( float *dest, float const *lhs, float const *rhs, F const& f )
            {
                (*dest++) = f(*lhs++, *rhs++);
                *dest = f(*lhs, *rhs);
            }
            
            template<typename F>
            static void Do( float *dest, float const *lhs, float const value, F const& f )
            {
                (*dest++) = f(*lhs++, value);
                *dest = f(*lhs, value);
            }*/
            
            static void Do( float *dest, float const *lhs, float const *rhs, std::plus<float> )
            {
                float32x2_t v = vadd_f32(*(float32x2_t *)lhs,
                                         *(float32x2_t *)rhs);
                
                *((float32x2_t*)dest) = v;
            }
            
            static void Do( float *dest, float const* lhs, float const value, std::plus<float> )
            {
                float32x2_t v = vadd_f32(*(float32x2_t *)lhs,
                                         vdup_n_f32((float32_t)value));
                
                *((float32x2_t*)dest) = v;
            }
            
            static void Do( float *dest, float const *lhs, float const *rhs, std::minus<float> )
            {
                float32x2_t v = vsub_f32(*(float32x2_t *)lhs,
                                         *(float32x2_t *)rhs);
                
                *((float32x2_t*)dest) = v;
            }
            
            static void Do( float *dest, float const* lhs, float const value, std::minus<float> )
            {
                float32x2_t v = vsub_f32(*(float32x2_t *)lhs,
                                         vdup_n_f32((float32_t)value));
                
                *((float32x2_t*)dest) = v;
            }
            
            
            static void Do( float *dest, float const *lhs, float const *rhs, std::multiplies<float> )
            {
                float32x2_t v = vmul_f32(*(float32x2_t *)lhs,
                                         *(float32x2_t *)rhs);
                
                *((float32x2_t*)dest) = v;
            }
            
            static void Do( float *dest, float const* lhs, float const value, std::multiplies<float> )
            {
                float32x2_t v = vmul_f32(*(float32x2_t *)lhs,
                                         vdup_n_f32((float32_t)value));
                
                *((float32x2_t*)dest) = v;
            }
            
            static void Do( float *dest, float const *lhs, float const *rhs, std::divides<float> )
            {
                float32x2_t *vLeft = (float32x2_t *)lhs;
                float32x2_t *vRight = (float32x2_t *)rhs;
                float32x2_t estimate = vrecpe_f32(*vRight);
                estimate = vmul_f32(vrecps_f32(*vRight, estimate), estimate);
                estimate = vmul_f32(vrecps_f32(*vRight, estimate), estimate);
                float32x2_t v = vmul_f32(*vLeft, estimate);
                
                *((float32x2_t*)dest) = v;
            }
            
            static void Do( float *dest, float const* lhs, float const value, std::divides<float> )
            {
                float32x2_t values = vdup_n_f32((float32_t)value);
                float32x2_t estimate = vrecpe_f32(values);
                estimate = vmul_f32(vrecps_f32(values, estimate), estimate);
                estimate = vmul_f32(vrecps_f32(values, estimate), estimate);
                float32x2_t v = vmul_f32(*(float32x2_t *)lhs, estimate);
                *((float32x2_t*)dest) = v;
            }
            
            static void Do( float *dest, float const *lhs, float const *rhs, lerp_hlp<float> lh )
            {
                float32x2_t vDiff = vsub_f32(*(float32x2_t *)rhs,
                                             *(float32x2_t *)lhs);
                vDiff = vmul_f32(vDiff, vdup_n_f32((float32_t)lh.get_time()));
                float32x2_t v = vadd_f32(*(float32x2_t *)lhs, vDiff);
                
                *((float32x2_t*)dest) = v;
            }
        };
        
        template<>
        struct accum_op<float, 2>
        {
            /*
            template<typename F>
            static float Do( float const *lhs, float const *rhs, F const& f )
            {
                return f((*lhs++), (*rhs++)) + f(*lhs, *rhs);
            }*/
            
            static float Do( float const *lhs, float const *rhs, std::multiplies<float> )
            {
                float32x2_t v = vmul_f32(*(float32x2_t *)lhs,
                                         *(float32x2_t *)rhs);
                v = vpadd_f32(v, v);
                return vget_lane_f32(v, 0);
            }
            
            static float Do( float const *lhs, float const *rhs, sqr_diff<float> )
            {
                float32x2_t diff = vsub_f32(*(float32x2_t *)lhs,
                                            *(float32x2_t *)rhs);
                
                float32x2_t v = vmul_f32(diff, diff);
                v = vpadd_f32(v, v);
                
                return vget_lane_f32(v, 0);
            }
            
        };


    } // namespace nInternal
    
} // namespace nMath

