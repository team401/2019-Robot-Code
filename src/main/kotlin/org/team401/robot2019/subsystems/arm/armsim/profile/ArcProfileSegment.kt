package org.team401.armsim.profile

import org.team401.armsim.ArmKinematics
import org.team401.armsim.Point2d
import kotlin.math.cos
import kotlin.math.sin

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
class ArcProfileSegment(override val start: Point2d,
                        override val end: Point2d,
                        private val radius: Double): ProfileSegment {

    private val startAngle = ArmKinematics.inverse(start).theta
    private val endAngle = ArmKinematics.inverse(end).theta

    init {
        println(Math.toDegrees(startAngle))
        println(Math.toDegrees(endAngle))
    }

    override fun solve(theta: Double): Point2d {
        val x = radius * cos(theta)
        val y = radius * sin(theta)
        return Point2d(x, y)
    }
}