package org.team401.robot2019.vision2

import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import kotlin.math.pow

/**
 * Implements the regression used to calculate the linear distance from a Limelight camera to a vision target,
 * given the area of the target as a percentage of the frame.
 *
 * The regression used is a power regression, and should be solved using external tools (such as Excel)
 */
class VisionRegression(val coeff: Double, val power: Double) {
    /**
     * Interpolates the distance from the target given the area of the target as a percentage
     */
    fun interpolate(targetArea: Double): Double {
        return coeff * targetArea.pow(power)
    }
}