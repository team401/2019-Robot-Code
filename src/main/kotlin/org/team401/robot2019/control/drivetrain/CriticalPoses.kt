package org.team401.robot2019.control.drivetrain

import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.util.fieldMirror
import org.team401.robot2019.util.withHeading
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import org.team401.taxis.trajectory.TrajectoryUtil
import kotlin.math.abs

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 */
object CriticalPoses {
    private fun flipWaypoints(waypointsIn: List<Pose2d>): List<Pose2d> {
        val waypoints = ArrayList<Pose2d>(waypointsIn.size)
        waypointsIn.forEach {
            val poseMirrored = Pose2d(it.translation.x(), -it.translation.y(), it.rotation.inverse())
            waypoints.add(poseMirrored)
        }

        return waypoints
    }
    
    const val habDriveDistance = 40.775

    val robotRelativeStart = Pose2d.identity()
    val robotRelativeDriveFromL2 = robotRelativeStart.transformBy(
        Pose2d.fromTranslation(Translation2d(habDriveDistance, 0.0))
    )
    val robotRelativeToRocketLeft = robotRelativeDriveFromL2.transformBy(
        Pose2d(101.216, 98.884, Rotation2d.fromDegrees(28.77))
    )

    val robotRelativeDriveToRocketLeftAlignEnd = robotRelativeToRocketLeft.transformBy(
        Pose2d.fromTranslation(Translation2d(-4.0 * 12.0, 0.0))
    )

    val stationRelativeLoadingStation = Pose2d.identity()
    val stationRelativeToRocketLeft = stationRelativeLoadingStation.transformBy(
        Pose2d(198.650, 1.063, Rotation2d.fromDegrees(28.77))
    )

    val stationRelativeToRocketLeftBackUp = stationRelativeToRocketLeft.transformBy(
        Pose2d.fromTranslation(Translation2d(-3.0 * 12.0, 16.0))
    ).withHeading(Rotation2d.identity())

    val stationRelativeToRocketLeftAlignEnd = stationRelativeToRocketLeft.transformBy(
        Pose2d.fromTranslation(Translation2d(-4.0 * 12.0, 0.0))
    )

    val stationRelativeToStationAlignStart = stationRelativeLoadingStation.transformBy(
        Pose2d.fromTranslation(Translation2d(8.0 * 12.0, 0.0))
    )
    val stationRelativeToStationAlignEnd = stationRelativeLoadingStation.transformBy(
        Pose2d.fromTranslation(Translation2d(4.0 * 12.0, 0.0))
    )

    val stationRelativeToHatchGrabbed = stationRelativeLoadingStation.transformBy(
        Pose2d.fromTranslation(Translation2d(38.0, 0.0))
    )

    val stationRelativeTurnPoint = stationRelativeToHatchGrabbed.transformBy(
        Pose2d.fromTranslation(Translation2d(5.0 * 12.0, -3.0 * 12.0))
    ).withHeading(Rotation2d.fromDegrees(-20.0))


    val habToRocketLeftPoints = listOf(
        robotRelativeStart,
        robotRelativeDriveFromL2,
        robotRelativeDriveToRocketLeftAlignEnd
    )

    val rocketLeftToStationPoints = listOf(
        stationRelativeToRocketLeft,
        stationRelativeToRocketLeftBackUp,
        stationRelativeToStationAlignStart,
        stationRelativeToStationAlignEnd
    )

    val stationToRocketLeftPoints = listOf(
        stationRelativeToHatchGrabbed,
        stationRelativeTurnPoint,
        stationRelativeToRocketLeftAlignEnd.transformBy(Pose2d.fromTranslation(Translation2d(0.0, -4.0)))
    )

    val habToRocketRightPoints = flipWaypoints(habToRocketLeftPoints)
    val rocketRightToStationPoints = flipWaypoints(rocketLeftToStationPoints)
    val stationToRocketRightPoints = flipWaypoints(stationToRocketLeftPoints)
}

fun genPointJson(pose: Pose2d): String {
    return """
        {
            'x': ${pose.translation.x() / 12.0},
            'y': ${pose.translation.y() / 12.0},
            'angle': ${pose.rotation.degrees}
        }
    """.trimIndent()
}

fun genJson(poses: List<Pose2d>): String {
    val jsonStrings = poses.joinToString { genPointJson(it) }
    return """
        [
        $jsonStrings,
          {
            "maxVoltage": 9,
            "maxVelocity": 36,
            "maxAcceleration": 36,
            "reverse": false,
            "maxCentripetalAcceleration": 0
          }
        ]
    """.trimIndent()
}

fun main(args: Array<String>) {
    CriticalPoses.stationToRocketLeftPoints.forEach {
        println(it)
    }

    println()

    CriticalPoses.stationToRocketRightPoints.forEach {
        println(it)
    }
}