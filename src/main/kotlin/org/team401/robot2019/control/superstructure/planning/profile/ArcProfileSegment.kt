package org.team401.robot2019.control.superstructure.planning.profile

import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
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
        //println(Math.toDegrees(startAngle))
        //println(Math.toDegrees(endAngle))
    }

    override fun solve(theta: AngularDistanceMeasureRadians): Point2d {
        val x = radius * cos(theta.value)
        val y = radius * sin(theta.value)
        return Point2d(x.Inches, y.Inches)
    }
}