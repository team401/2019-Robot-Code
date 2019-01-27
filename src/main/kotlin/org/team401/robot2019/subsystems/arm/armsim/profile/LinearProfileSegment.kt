package org.team401.armsim.profile

import org.team401.armsim.Point2d
import java.lang.Math.pow
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
class LinearProfileSegment(override val start: Point2d,
                           override val end: Point2d): ProfileSegment {

    private val m = (end.y - start.y) / (end.x - start.x)
    private val b = start.y - start.x * m

    override fun solve(theta: Double): Point2d { // For graphing ONLY
        var x = Double.NaN
        var y = Double.NaN
        if (!m.isFinite()){
            x = start.x
            y = x * tan(theta)
        }else {
            x = b / (tan(theta) - m)
            y = tan(theta) * b / (tan(theta) - m)
        }
        return Point2d(x, y)
    }

    fun distance(): Double{
        return sqrt(pow(end.x - start.x, 2.0) + pow(end.y - start.y, 2.0))
    }

    fun getM(): Double{
        return m
    }

    fun getB():Double{
        return b
    }
}