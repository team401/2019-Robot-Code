package org.team401.robot2019.control.superstructure.geometry

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians

data class WristState(val wristPosition: AngularDistanceMeasureRadians,
                      val hasCargo: Boolean,
                      val hasHatchPanel: Boolean)