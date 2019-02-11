package org.team401.robot2019.control.superstructure.planning.profile

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond

data class TrapezoidalProfilePoint(val position: AngularDistanceMeasureRadians,
                                   val velocity: AngularVelocityMeasureRadiansPerSecond,
                                   val time: TimeMeasureSeconds
)