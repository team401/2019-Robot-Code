package org.team401.robot2019.subsystems.arm

import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.units.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.armsim.PointPolar

data class ArmState(val position: Pair<LinearDistanceMeasureInches, AngularDistanceMeasureRadians>,
                    val armVelocity: AngularVelocityMeasureRadiansPerSecond)