package org.team401.robot2019.subsystems.arm.armsim.profile

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.subsystems.arm.armsim.Point2d

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
interface ProfileSegment {
    val start: Point2d
    val end: Point2d

    /**
     * Solves for the system coordinates at percentage of profile p
     */
    fun solve(theta: AngularDistanceMeasureRadians): Point2d
}