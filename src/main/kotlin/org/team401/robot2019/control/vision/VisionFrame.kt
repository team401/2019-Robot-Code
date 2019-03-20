package org.team401.robot2019.control.vision

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Milliseconds
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.measure.time.TimeMeasureMilliseconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

/**
 * Represents a single frame from the vision system.
 */
data class VisionFrame(val timeReceived: TimeMeasureSeconds,
                       val hasTarget: Boolean,
                       val targetAngleHoriz: AngularDistanceMeasureDegrees,
                       val targetAngleVert: AngularDistanceMeasureDegrees,
                       val latency: TimeMeasureMilliseconds,
                       val activePipeline: Int,
                       val x: LinearDistanceMeasureInches,
                       val y: LinearDistanceMeasureInches,
                       val z: LinearDistanceMeasureInches,
                       val pitch: AngularDistanceMeasureDegrees,
                       val yaw: AngularDistanceMeasureDegrees,
                       val roll: AngularDistanceMeasureDegrees) {
    val timeCaptured = (timeReceived - latency).toSeconds()

    fun toPose2d(): Pose2d {
        return Pose2d(z.value, -x.value, Rotation2d.fromDegrees(yaw.value))
    }

    companion object {
        fun identity(): VisionFrame {
            return VisionFrame(
                0.0.Seconds,
                false,
                0.0.Degrees,
                0.0.Degrees,
                0.0.Milliseconds,
                -1,
                0.0.Inches,
                0.0.Inches,
                0.0.Inches,
                0.0.Degrees,
                0.0.Degrees,
                0.0.Degrees
            )
        }
    }
}