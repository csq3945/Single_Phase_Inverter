#include "myself_power_transform.h"

/**
 * @brief Equal amplitudes Clarke Transform, a equal alpha
 * @param threephase Structure storing Three-phase data.
 * @param &clarke The address of the structure storing Clarke data
 * @retval None
 */
inline void Clarke_Transform(ThreePhase threephase, Clarke *clarke)
{
    clarke->alpha = threephase.a;
    clarke->beta = (2 * threephase.b + threephase.a) / 1.732051f;
}

/**
 * @brief Equal amplitudes Inverse Clarke Transform
 * @param clarke Structure storing Clarke data.
 * @param &threephase The address of the structure storing Three-phase data.
 * @retval None
 */
inline void Clarke_Transform_Inverse(Clarke clarke, ThreePhase *threephase)
{
    threephase->a = clarke.alpha;
    threephase->b = (1.732051f * clarke.beta - clarke.alpha) / 2.f;
    threephase->c = 0 - threephase->a - threephase->b;
}

/**
 * @brief Park Transform based on cos, d start from alpha
 * @param clarke Structure storing Clarke data.
 * @param &park The address of the structure storing Park data.
 * @param theta The transforming electrical angle(rad).
 * @retval None
 */
inline void Park_Transform(Clarke clarke, Park *park, float theta)
{
    park->d = clarke.alpha * cosf(theta) + clarke.beta * sinf(theta);
    park->q = clarke.beta * cosf(theta) - clarke.alpha * sinf(theta);
}

/**
 * @brief Inverse Park Transform based on cos
 * @param park Structure storing Park data.
 * @param &clarke The address of the structure storing Clarke data.
 * @param theta The transforming electrical angle(rad).
 * @retval None
 */
inline void Park_Transform_Inverse(Park park, Clarke *clarke, float theta)
{
    clarke->alpha = park.d * cosf(theta) - park.q * sinf(theta);
    clarke->beta = park.q * cosf(theta) - park.d * sinf(theta);
}


