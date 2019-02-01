package org.team401.robot2019.subsystems.arm

import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.units.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.armsim.PointPolar

/**
 * Holds the state of the arm as an effective polar point + other stuff.
 * Radius is from the pivot, not a measure of extension
 */
data class ArmState(val armRadius: LinearDistanceMeasureInches,
                    val armAngle: AngularDistanceMeasureRadians,
                    val armVelocity: AngularVelocityMeasureRadiansPerSecond)