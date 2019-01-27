package org.team401.armsim

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
            point.r * Math.cos(point.theta),
            point.r * Math.sin(point.theta)
        )
    }

    /**
     * Solves inverse kinematics, which converts rectangular domain endpoint positions to polar domain joint positions
     */
    fun inverse(point: Point2d): PointPolar {
        var angle = Math.atan2(point.y, point.x)
        if (angle <= -(Math.PI / 2.0)) {
            angle += Math.PI * 2.0
        }
        return PointPolar(
            Math.hypot(point.x, point.y),
            angle
        )
    }
}