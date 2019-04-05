package org.team401.robot2019.subsystems.arm.control

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.Unitless
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.PointPolar

/**
 * @author Cameron Earle
 * @version 1/20/2019
 *
 */
object ArmKinematics {
    /**
     * Solves forward kinematics, which converts polar domain joint positions to rectangular domain endpoint positions
     */
    fun forward(point: PointPolar): Point2d {
        return Point2d(
            (point.r * Math.cos(point.theta.value).Unitless),
            (point.r * Math.sin(point.theta.value).Unitless)
        )
    }

    fun forward(point: ArmState): Point2d {
        return Point2d(
            (point.armRadius * Math.cos(point.armAngle.value).Unitless),
            (point.armRadius * Math.sin(point.armAngle.value).Unitless)
        )
    }

    /**
     * Solves inverse kinematics, which converts rectangular domain endpoint positions to polar domain joint positions
     */
    fun inverse(point: Point2d): PointPolar {
        var angle = Math.atan2(point.y.value, point.x.value)
        if (angle <= -(Math.PI / 2.0)) {
            angle += Math.PI * 2.0
        }
        return PointPolar(
            Math.hypot(point.x.value, point.y.value).Inches,
            angle.Radians
        )
    }
}

fun main(args: Array<String>) {
    println(ArmKinematics.inverse(Point2d(27.0.Inches, (-4.0).Inches)))
}