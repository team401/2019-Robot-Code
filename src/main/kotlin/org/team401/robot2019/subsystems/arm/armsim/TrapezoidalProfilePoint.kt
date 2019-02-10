package org.team401.robot2019.subsystems.arm.armsim

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond

data class TrapezoidalProfilePoint(val position: AngularDistanceMeasureRadians,
                                   val velocity: AngularVelocityMeasureRadiansPerSecond,
                                   val time: TimeMeasureSeconds
)