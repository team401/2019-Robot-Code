package org.team401.robot2019.control.climbing

import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.linear.LinearVelocityMeasureInchesPerSecond

/**
 * Similar to the version in the superstructure planner but with linear units for climbing
 */
data class ClimbingProfilePoint(val position: LinearDistanceMeasureInches,
                                   val velocity: LinearVelocityMeasureInchesPerSecond,
                                   val time: TimeMeasureSeconds
)