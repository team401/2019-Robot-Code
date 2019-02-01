package org.team401.armsim

import org.snakeskin.units.Inches
import org.snakeskin.units.Radians
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.subsystems.arm.ArmState

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
            (point.r * Math.cos(point.theta.value)) as LinearDistanceMeasureInches,
            (point.r * Math.sin(point.theta.value)) as LinearDistanceMeasureInches
        )
    }

    fun forward(point: ArmState): Point2d {
        return Point2d(
            (point.position.first * Math.cos(point.position.second.value)) as LinearDistanceMeasureInches,
            (point.position.first * Math.sin(point.position.second.value)) as LinearDistanceMeasureInches
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