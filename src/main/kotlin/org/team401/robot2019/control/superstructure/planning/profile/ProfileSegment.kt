package org.team401.robot2019.control.superstructure.planning.profile

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.control.superstructure.geometry.Point2d

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