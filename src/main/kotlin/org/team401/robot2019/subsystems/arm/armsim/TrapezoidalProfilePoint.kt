package org.team401.robot2019.subsystems.arm.armsim

import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.time.TimeMeasureSeconds
import org.snakeskin.units.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import java.text.FieldPosition

data class TrapezoidalProfilePoint(val position: AngularDistanceMeasureRadians,
                                   val velocity: AngularVelocityMeasureRadiansPerSecond,
                                   val time: TimeMeasureSeconds)