package org.team401.robot2019.subsystems.arm

import org.snakeskin.units.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.armsim.PointPolar

data class ArmState(val position: PointPolar, val armVelocity: AngularVelocityMeasureRadiansPerSecond)