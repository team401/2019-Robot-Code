package org.team401.robot2019.subsystems.arm.control

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches

/**
 * @param armAngle The angle of the arm's pivot, in radians
 * @param armRadius The total radius of the arm (including the constant lower radius) in inches
 * @param wristTheta The angle of the arm's pivot relative to the plane formed by the end of the arm
 * @param armFeedForwardVoltage The total feed-forward voltage, in volts, to be applied to the arm.  Maximum 12.0v
 */
data class SuperstructureControlOutput(val armAngle: AngularDistanceMeasureRadians,
                                       val armRadius: LinearDistanceMeasureInches,
                                       val wristTheta: AngularDistanceMeasureRadians,
                                       val armFeedForwardVoltage: Double)
