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

#include "../math.hpp"

namespace nMath {
namespace nNNet {

template<typename T>
struct DefaultLearnHelper
{
    DefaultLearnHelper( T learn_rate = static_cast<T>(0.212) )
        :
        learn_rate_(learn_rate)
        {}
    T const& get_learn_rate()     { return learn_rate_; }
    static T get_rand_weight()
    {
        return static_cast<T>
        (
            static_cast<T>(rand()) / static_cast<T>(RAND_MAX)
            -
            cHalf<T>()
        );
    }
private:
    T       learn_rate_;
};

template<typename T>
struct DefaultSigmoidHelper
{
    static T sigmoid( const T& val )
    {
        return cOne<T>() / ( cOne<T>() + std::exp(-val) );
    }

    static T sigmoid_derivative( const T& val )
    {
        return val * ( cOne<T>() - val );
    }
};

template
<
    typename                T,
    int                     INPUT_NEURONS,
    int                     HIDDEN_NEURONS,
    int                     OUTPUT_NEURONS,
    template<class> class   SigmoidHelper   = DefaultSigmoidHelper,
    template<class> class   LearnHelper     = DefaultLearnHelper
>
class WinnerGetsAll_NeuralNET
    :
    private LearnHelper<T>,
    private SigmoidHelper<T>
{
    typedef const T (&const_ref_wih_array)[INPUT_NEURONS + 1][HIDDEN_NEURONS];
    typedef const T (&const_ref_who_array)[HIDDEN_NEURONS + 1][OUTPUT_NEURONS];
public:
    struct LearnData
    {
        T in[INPUT_NEURONS],
          out[OUTPUT_NEURONS]
          ;
    };
    enum
    {
        eInputNeurons   = INPUT_NEURONS,
        eHiddenNeurons  = HIDDEN_NEURONS,
        eOutputNeurons  = OUTPUT_NEURONS
    };
public:
    WinnerGetsAll_NeuralNET( LearnHelper<T> lh = LearnHelper<T>() )
        :
        LearnHelper<T>(lh)
        {}

    const_ref_wih_array get_wih_array() const
    {
        return wih_;
    }
    const_ref_who_array get_who_array() const
    {
        return who_;
    }
    template<typename IterT>
    void begin_learn( IterT it0, IterT it1, int nIterations )
    {
        assign_random_weights();

        for( int i = 0; i < nIterations; ++i )
        {
            for( IterT it = it0; it != it1; ++it )
            {
                feed_forward(it->in);
                back_propagate(it->in, it->out);
            }
        }
    }

    template<typename IterT>
    // return number of true answers
    int test_network( IterT it0, IterT it1 )
    {
        int _true = 0;
        for( ; it0 != it1; ++it0 )
        {
            feed_forward(it0->in);
            if( calc_max_pos(it0->out) == calc_max_pos(actual_) )
                _true++;
        }

        return _true;
    }

private:
    using LearnHelper<T>::get_learn_rate;
    using LearnHelper<T>::get_rand_weight;

    int calc_max_pos( const T (&vector)[OUTPUT_NEURONS] )
    {
        T max   = vector[0];
        int pos = 0;

        for( int i = 1; i < OUTPUT_NEURONS; i++ )
        {
            if( vector[i] > max )
            {
                max = vector[i];
                pos = i;
            }
        }

        return pos;
    }

    void feed_forward( const T (&inputs)[INPUT_NEURONS] )
    {
        // Calculate input to hidden_ layer
        for( int hid = 0; hid < HIDDEN_NEURONS; hid++ )
        {
            T sum = cZero<T>();
            for( int inp = 0; inp < INPUT_NEURONS; inp++ )
            {
                sum += inputs[inp] * wih_[inp][hid];
            }
            // Add in Bias
            sum += wih_[INPUT_NEURONS][hid];

            hidden_[hid] = sigmoid( sum );
        }

        // Calculate the hidden_ to output layer
        for( int out = 0 ; out < OUTPUT_NEURONS; out++ )
        {
            T sum = cZero<T>();
            for( int hid = 0; hid < HIDDEN_NEURONS; hid++ )
            {
                sum += hidden_[hid] * who_[hid][out];
            }
            // Add in Bias
            sum += who_[HIDDEN_NEURONS][out];

            actual_[out] = sigmoid( sum );
        }
    }

    void back_propagate
    (
        const T (&inputs)[INPUT_NEURONS],
        const T (&target)[OUTPUT_NEURONS]
    )
    {
        T   erro[OUTPUT_NEURONS],
            errh[HIDDEN_NEURONS]
            ;
        // Calculate the output layer error (step 3 for output cell)
        for( int out = 0; out < OUTPUT_NEURONS; out++ )
        {
            erro[out] = (target[out] - actual_[out]) * sigmoid_derivative( actual_[out] );
        }

        // Calculate the hidden_ layer error (step 3 for hidden cell)
        for( int hid = 0; hid < HIDDEN_NEURONS; hid++ )
        {
            errh[hid] = cZero<T>();
            for( int out = 0; out < OUTPUT_NEURONS; out++ )
            {
                errh[hid] += erro[out] * who_[hid][out];
            }

            errh[hid] *= sigmoid_derivative( hidden_[hid] );
        }

        // Update the weights for the output layer (step 4 for output cell)
        for( int out = 0; out < OUTPUT_NEURONS; out++ )
        {
            for( int hid = 0; hid < HIDDEN_NEURONS; hid++ )
            {
                who_[hid][out] += (get_learn_rate() * erro[out] * hidden_[hid]);
            }

            // Update the Bias
            who_[HIDDEN_NEURONS][out] += (get_learn_rate() * erro[out]);
        }

        // Update the weights for the hidden_ layer (step 4 for hidden cell)
        for( int hid = 0; hid < HIDDEN_NEURONS; hid++ )
        {
            for( int inp = 0; inp < INPUT_NEURONS; inp++ )
            {
                wih_[inp][hid] += (get_learn_rate() * errh[hid] * inputs[inp]);
            }

            // Update the Bias
            wih_[INPUT_NEURONS][hid] += (get_learn_rate() * errh[hid]);
        }
    }

    void assign_random_weights()
    {
        for( int inp = 0; inp < INPUT_NEURONS + 1; inp++ )
        {
            for( int hid = 0; hid < HIDDEN_NEURONS; hid++ )
            {
                wih_[inp][hid] = get_rand_weight();
            }
        }

        for( int hid = 0; hid < HIDDEN_NEURONS + 1; hid++ )
        {
            for( int out = 0; out < OUTPUT_NEURONS; out++ )
            {
                who_[hid][out] = get_rand_weight();
            }
        }
    }
private:
    T       wih_[INPUT_NEURONS + 1][HIDDEN_NEURONS],
            who_[HIDDEN_NEURONS + 1][OUTPUT_NEURONS],
            hidden_[HIDDEN_NEURONS],
            actual_[OUTPUT_NEURONS]
            ;
};

template<typename T> struct NoLearnAbility {};

template
<
    typename                T,
    int                     INPUT_NEURONS,
    int                     HIDDEN_NEURONS,
    int                     OUTPUT_NEURONS,
    template<class> class   SigmoidHelper
>
class WinnerGetsAll_NeuralNET
    <
        T,
        INPUT_NEURONS,
        HIDDEN_NEURONS,
        OUTPUT_NEURONS,
        SigmoidHelper,
        NoLearnAbility
    >
    :
    private SigmoidHelper<T>
{
public:
    struct LearnData
    {
        T in[INPUT_NEURONS],
          out[OUTPUT_NEURONS]
          ;
    };
public:
    template<typename IterT>
    // wih size must be INPUT_NEURONS*HIDDEN_NEURONS
    // who size must be HIDDEN_NEURONS*OUTPUT_NEURONS
    WinnerGetsAll_NeuralNET( IterT begin, IterT end )
    {
        if
        (
            std::distance(begin, end) !=
            ((INPUT_NEURONS + 1)*HIDDEN_NEURONS + (HIDDEN_NEURONS + 1)*OUTPUT_NEURONS) 
        )
        {
            throw std::runtime_error("Invalid nnet weights size passed!");
        }
        for( int i = 0; i < (INPUT_NEURONS + 1); ++i )
        {
            for( int h = 0; h < HIDDEN_NEURONS; ++h )
                wih_[i][h] = (*begin++);
        }
        for( int h = 0; h < (HIDDEN_NEURONS + 1); ++h )
        {
            for( int o = 0; o < OUTPUT_NEURONS; ++o )
                who_[h][o] = (*begin++);
        }
    }

    template<typename IterT>
    // return number of true answers
    int test_network( IterT it0, IterT it1 )
    {
        int _true = 0;
        for( ; it0 != it1; ++it0 )
        {
            feed_forward(it0->in);
            if( calc_max_pos(it0->out) == calc_max_pos(actual_) )
                _true++;
        }

        return _true;
    }

    T const& operator [] ( int idx ) const
    {
        assert( idx < OUTPUT_NEURONS && "Output neuron index overbound!" );
        return actual_[idx];
    }

    int calc_max_pos()
    {
        T max   = actual_[0];
        int pos = 0;

        for( int i = 1; i < OUTPUT_NEURONS; i++ )
        {
            if( actual_[i] > max )
            {
                max = actual_[i];
                pos = i;
            }
        }

        return pos;
    }

    void feed_forward( const T (&inputs)[INPUT_NEURONS] )
    {
        // Calculate input to hidden_ layer
        for( int hid = 0; hid < HIDDEN_NEURONS; hid++ )
        {
            T sum = cZero<T>();
            for( int inp = 0; inp < INPUT_NEURONS; inp++ )
            {
                sum += inputs[inp] * wih_[inp][hid];
            }
            // Add in Bias
            sum += wih_[INPUT_NEURONS][hid];

            hidden_[hid] = sigmoid( sum );
        }

        // Calculate the hidden_ to output layer
        for( int out = 0 ; out < OUTPUT_NEURONS; out++ )
        {
            T sum = cZero<T>();
            for( int hid = 0; hid < HIDDEN_NEURONS; hid++ )
            {
                sum += hidden_[hid] * who_[hid][out];
            }
            // Add in Bias
            sum += who_[HIDDEN_NEURONS][out];

            actual_[out] = sigmoid( sum );
        }
    }
private:
    T       wih_[INPUT_NEURONS + 1][HIDDEN_NEURONS];
    T       who_[HIDDEN_NEURONS + 1][OUTPUT_NEURONS];
    T       hidden_[HIDDEN_NEURONS],
            actual_[OUTPUT_NEURONS]
            ;
};


} // namespace nNNet

} // namespcace nMath
