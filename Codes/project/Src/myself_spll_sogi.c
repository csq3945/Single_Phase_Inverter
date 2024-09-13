#include "myself_spll_sogi.h"

void SOGI_Init(Sogi *sogi)
{
    sogi->clarke_last.alpha = 0;
    sogi->clarke_last.beta = 0;

    sogi->clarke.alpha = 0;
    sogi->clarke.beta = 0;

    sogi->gain = 1;
    sogi->temp_a = 0;
    sogi->temp_ai = 0;
    sogi->temp_bi = 0;
}

void PLL_Init(Pll *pll)
{
    pll->park.d = 0;
    pll->park.q = 0;

    pll->lock_theta = 0;
    pll->lock_w = 2*PI/400;

    PID_Position_Init(&(pll->pid), 0, 0.01, 0.000001, 0);
}

void SPLL_Init(Spll *spll)
{
    SOGI_Init(&(spll->sogi));
    PLL_Init(&(spll->pll));
}

inline void SOGI_Run(Sogi *sogi, SinglePhase singlephase)
{
    sogi->temp_a = singlephase.value - sogi->clarke_last.alpha;
    sogi->temp_a *= sogi->gain;
    sogi->temp_a -= sogi->clarke_last.beta;

    sogi->temp_ai += sogi->temp_a;
    sogi->clarke.alpha = sogi->temp_ai * singlephase.w;

    sogi->temp_bi += sogi->clarke_last.alpha;
    sogi->clarke.beta = sogi->temp_bi * singlephase.w;

    sogi->clarke_last.alpha = sogi->clarke.alpha;
    sogi->clarke_last.beta = sogi->clarke.beta;
}

inline void PLL_Run(Pll *pll, Clarke clarke, float w)
{
    Park_Transform(clarke, &(pll->park), pll->lock_theta);

    pll->lock_w = w - PID_Position_Run(&(pll->pid), pll->park.q);
    pll->lock_theta += pll->lock_w;
    pll->lock_theta = fmod(pll->lock_theta, 2*PI);
}

/**
 * @brief SPLL
 * @param *spll Pointer to Spll struct
 * @param sample_value The instantaneous value of the sample
 * @param input_w The angular frequency of the sampled waveform
 * @retval Phase-locked sinusoidal waveform
 */
inline float Sample_Phase_locked_loop(Spll *spll, SinglePhase input)
{
    SOGI_Run(&(spll->sogi), input);
    PLL_Run(&(spll->pll), spll->sogi.clarke, input.w);
    return cosf(spll->pll.lock_theta);
}





