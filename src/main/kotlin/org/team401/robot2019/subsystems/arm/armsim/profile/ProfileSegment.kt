package org.team401.armsim.profile

import org.team401.armsim.Point2d

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
    fun solve(theta: Double): Point2d
}