package org.team401.armsim

import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches

/**
 * @author Cameron Earle
 * @version 1/20/2019
 *
 */
data class PointPolar(val r: LinearDistanceMeasureInches, val theta: AngularDistanceMeasureRadians)