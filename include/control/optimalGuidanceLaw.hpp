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
 * Computes the control authority based on the OGL (Ebrahimi et al., 2008; Furfaro et al., 2011;
 * Guo et al., 2012; Guo et al., 2013). The OGL is the optimal control authority for the case
 * of constant gravity. The control authority is given by the following equation:
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
    Vector3 controlEffort = zeroEffortMiss;

    const Real zeroEffortMissPremultiplier     = zeroEffortMissGain / ( timeToGo * timeToGo );
    const Real zeroEffortVelocityPremultiplier = zeroEffortVelocityGain / timeToGo;

    controlEffort[ 0 ] = zeroEffortMissPremultiplier * zeroEffortMiss[ 0 ]
                        + zeroEffortVelocityPremultiplier * zeroEffortVelocity[ 0 ];
    controlEffort[ 1 ] = zeroEffortMissPremultiplier * zeroEffortMiss[ 1 ]
                        + zeroEffortVelocityPremultiplier * zeroEffortVelocity[ 1 ];
    controlEffort[ 2 ] = zeroEffortMissPremultiplier * zeroEffortMiss[ 2 ]
                        + zeroEffortVelocityPremultiplier * zeroEffortVelocity[ 2 ];

    return controlEffort;
}

} // namespace control

#endif // CONTROL_OPTIMAL_GUIDANCE_LAW_HPP

/*
 * References
 * Ebrahimi, B., Bahrami, M., Roshanian, J. (2008) Optimal sliding-mode guidance with terminal
 *  velocity constraint for fixed-interval propulsive maneuvers, Acta Astronautica, pg. 556–562,
 *  vol. 62, doi: 10.1016/j.actaastro.2008.02.002.
 * Furfaro, R., Gaudet, B., Wibben, D.R. Simo, J. (2011) Development of Non-Linear Guidance
 *  Algorithms for Asteroids Close-Proximity Operations, AIAA Guidance, Navigation, and Control
 *  (GNC) Conference 2013, Boston, MA, doi: 10.2514/6.2013-4711.
 * Guo, Y., Hawkins, M., Wie, B. (2012) Optimal feedback guidance algorithms for planetary landing
 *  and asteroid intercept, Advances in the Astronautical Sciences, pg. 2913-2931, vol. 142.
 * Guo, Y., Hawkins, M., Wie, B. (2013) Applications of Generalized
 *  Zero-Effort-Miss/Zero-Effort-Velocity Feedback Guidance Algorithm, Journal of Guidance, Control,
 *  and Dynamics, pg. 810-820, vol. 36, doi: 10.2514/1.58099.
 * Furfaro
 */
