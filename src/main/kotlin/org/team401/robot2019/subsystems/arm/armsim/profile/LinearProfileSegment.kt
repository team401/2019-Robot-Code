package org.team401.robot2019.subsystems.arm.armsim.profile

import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.subsystems.arm.armsim.Point2d
import java.lang.Math.pow
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
class LinearProfileSegment(override val start: Point2d,
                           override val end: Point2d
): ProfileSegment {

    private val endX = end.x.value
    private val endY = end.y.value
    private val startX = start.x.value
    private val startY = start.y.value

    private val m = (endY - startY) / (endX - startX)
    private val b = startY - startX * m

    override fun solve(theta: AngularDistanceMeasureRadians): Point2d {
        var x: Double
        var y: Double
        if (!m.isFinite()){
            x = startX
            y = x * tan(theta.value)
        }else {
            x = b / (tan(theta.value) - m)
            y = tan(theta.value) * b / (tan(theta.value) - m)
        }
        return Point2d(x.Inches, y.Inches)
    }

    fun distance(): Double{
        return sqrt(pow(endX - startX, 2.0) + pow(endY - startY, 2.0))
    }

    fun getM(): Double{
        return m
    }

    fun getB():Double{
        return b
    }
}