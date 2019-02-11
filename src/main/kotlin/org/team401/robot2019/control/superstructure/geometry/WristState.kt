package org.team401.robot2019.control.superstructure.geometry

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

data class WristState(val wristPosition: AngularDistanceMeasureRadians,
                      val currentTool: WristMotionPlanner.Tool,
                      val hasCargo: Boolean,
                      val hasHatchPanel: Boolean)