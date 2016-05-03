/*
 * Copyright (c) 2016 Kartik Kumar, Dinamica Srl (me@kartikkumar.com)
 * Distributed under the MIT License.
 * See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
 */

#include <catch.hpp>

#include <Eigen/Core>

#include "Control/optimalGuidanceLaw.hpp"

namespace control
{
namespace tests
{

typedef double Real;
typedef Eigen::Matrix< Real, Eigen::Dynamic, 1 > Vector;
static const Real tolerance = 100.0 * std::numeric_limits< Real >::epsilon( );

TEST_CASE( "Test Optimal Guidance Law (OGL)", "[ogl]")
{
    // The tests presented here are only to confirm internal consistency by testing against a
    // snapshot of the results obtained at the time of writing the test.

    SECTION( "Test arbitrary case" )
    {
        const Real timeToGo = 12.516;

        Vector zeroEffortMiss( 3 );
        zeroEffortMiss[ 0 ] = -21.163;
        zeroEffortMiss[ 1 ] = 9.887;
        zeroEffortMiss[ 2 ] = -0.613;

        Vector zeroEffortVelocity( 3 );
        zeroEffortVelocity[ 0 ] = -1.244;
        zeroEffortVelocity[ 1 ] = -0.112;
        zeroEffortVelocity[ 2 ] = 3.119;

        Vector expectedControl( 3 );
        expectedControl[ 0 ] = 0.468979814498356;
        expectedControl[ 1 ] = -0.108333152037747;
        expectedControl[ 2 ] = -0.490575693665001;

        Vector computedControl = computeOptimalGuidanceLaw( zeroEffortMiss,
                                                            zeroEffortVelocity,
                                                            timeToGo );

        for ( unsigned int i = 0; i < 3; ++i )
        {
            REQUIRE( computedControl[ i ]
                        == Approx( expectedControl[ i ] ).epsilon( tolerance ) );
        }
    }
}

} // namespace tests
} // namespace control
