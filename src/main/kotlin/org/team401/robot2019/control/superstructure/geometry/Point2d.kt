package org.team401.robot2019.control.superstructure.geometry

import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.config.Geometry

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
data class Point2d(val x: LinearDistanceMeasureInches, val y: LinearDistanceMeasureInches) {
    class InvalidPointException(errorMessage: String): Exception(errorMessage)

    private val MAX_X = Geometry.ArmGeometry.maxX.value
    private val MIN_X = MAX_X * -1.0
    private val MAX_Y = Geometry.ArmGeometry.maxY.value
    private val MIN_Y = Geometry.ArmGeometry.minY.value

    init {
        if (!withinBounds(x.value, MIN_X, MAX_X)){
            throw InvalidPointException("X coordinate is out of bounds. x = ${x.value}")
        }
        if (!withinBounds(y.value, MIN_Y, MAX_Y)){
            throw InvalidPointException("Y coordinate is out of bounds. y = ${y.value}")
        }
    }

    override fun toString(): String {
        return "($x, $y)"
    }

    private fun withinBounds(value: Double, min: Double, max: Double):Boolean{
        return value >= min && value <= max
    }

}