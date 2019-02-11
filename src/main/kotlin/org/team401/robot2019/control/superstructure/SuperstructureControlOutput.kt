package org.team401.robot2019.control.superstructure

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches

/**
 * @param armAngle The angle of the superstructure's pivot, in radians
 * @param armRadius The total radius of the superstructure (including the constant lower radius) in inches
 * @param wristTheta The angle of the superstructure's pivot relative to the plane formed by the end of the superstructure
 * @param armFeedForwardVoltage The total feed-forward voltage, in volts, to be applied to the superstructure.  Maximum 12.0v
 */
data class SuperstructureControlOutput(val armAngle: AngularDistanceMeasureRadians,
                                       val armRadius: LinearDistanceMeasureInches,
                                       val wristTheta: AngularDistanceMeasureRadians,
                                       val armFeedForwardVoltage: Double)
