package org.team401.armsim
import java.lang.Math.pow
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
data class Point2d(val x: Double, val y: Double){
    // TODO Use actual numbers
    private val MAX_X = 8.0
    private val MIN_X = -MAX_X
    private val MAX_Y = 10.0
    private val MIN_Y = -2.0
    private val r = 4.0

    init {
        if (!withinBounds(x, MIN_X, MAX_X)){
            throw InvalidPointException("X coordinate is out of bounds")
        }
        if (!withinBounds(y, MIN_Y, MAX_Y)){
            throw InvalidPointException("Y coordinate is out of bounds")
        }

        if (withinCircle()){
            throw InvalidPointException("Point is within minimum circle")
        }
    }

    override fun toString(): String {
        return "($x, $y)"
    }

    private fun withinBounds(value: Double, min: Double, max: Double):Boolean{
        return value >= min && value <= max
    }
    private fun withinCircle(): Boolean{
        val value = sqrt(pow(x, 2.0) + pow(y, 2.0))
        if (abs(value - r) < 0.1){
            return false
        }
        return value < r
    }
}