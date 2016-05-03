/*
 * Copyright (c) 2016 Kartik Kumar, Dinamica Srl (me@kartikkumar.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#ifndef CONTROL_OPTIMAL_GUIDANCE_LAW_HPP
#define CONTROL_OPTIMAL_GUIDANCE_LAW_HPP

namespace control
{

//! Compute control authority for Optimal Guidance Law (OGL).
/*!
 * Computes the control authority based on the OGL (REF). The OGL is the optimal control authority
 * for the case of constant gravity. It is given by the following equation:
 *
 * \f[
 *      u(t) = \frac{k_{r}}{t_{\text{go}}^{2}} \vec{\text{ZEM}}(t)
 *              + \frac{k_{v}}{t_{\text{go}}} \vec{\text{ZEV}}(t)
 * \f]
 *
 * where \f$k_{r}=6.0\f$ and \f$k_{r}=-2.0\f$ are the gains for the optimal case,
 * \f$t_{\text{go}}\f$ is the Time-To-Go (TTG) to reach the target, \f$\vec{\text{ZEM}}\f$(t) is the
 * Zero-Effort-Miss vector derived from the difference between the target position
 * (\f$\vec{r}_{2}\f$) and the computed arrival position (\f$\tilde{\vec{r}}_{2}\f$), under the
 * assumption of no further control authority from the current time (\f$t_{1}\f$) to the final time
 * (\f$t_{2}\f$), and \f$\vec{\text{ZEV}}(t)\f$ is similarly the Zero-Effort-Velocity vector.
 *
 * @tparam  Real                   Real type
 * @tparam  Vector3                3-Vector type
 * @param   zeroEffortMiss         Miss distance vector between target and computed final state
 * @param   zeroEffortVelocity     Miss velocity vector between target and computed final state
 * @param   timeToGo               TTG to reach target
 * @param   zeroEffortMissGain     Control gain for ZEM term (default=6.0)
 * @param   zeroEffortVelocityGain Control gain for ZEV term (default=-2.0)
 * @return                         Computed control authority

 */
template< typename Real, typename Vector3 >
Vector3 computeOptimalGuidanceLaw( const Vector3& zeroEffortMiss,
                                   const Vector3& zeroEffortVelocity,
                                   const Real timeToGo,
                                   const Real zeroEffortMissGain = 6.0,
                                   const Real zeroEffortVelocityGain = -2.0 )
{
    Vector3 control = zeroEffortMiss;

    const Real zeroEffortMissPremultiplier     = zeroEffortVelocityGain / ( timeToGo * timeToGo );
    const Real zeroEffortVelocityPremultiplier = zeroEffortVelocityGain / timeToGo;

    control[ 0 ] = zeroEffortMissPremultiplier * zeroEffortMiss[ 0 ]
                   + zeroEffortVelocityPremultiplier * zeroEffortVelocity[ 0 ];
    control[ 1 ] = zeroEffortMissPremultiplier * zeroEffortMiss[ 1 ]
                   + zeroEffortVelocityPremultiplier * zeroEffortVelocity[ 1 ];
    control[ 2 ] = zeroEffortMissPremultiplier * zeroEffortMiss[ 2 ]
                   + zeroEffortVelocityPremultiplier * zeroEffortVelocity[ 2 ];

    return control;
}

} // namespace control

#endif // CONTROL_OPTIMAL_GUIDANCE_LAW_HPP
