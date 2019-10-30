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
    //Field relative positions of various field elements
    val fieldToNearRocketRight = Pose2d(213.678, 18.116, Rotation2d.fromDegrees(331.23))
    val fieldToInboundingStationRight = Pose2d(0.0, 25.715, Rotation2d.fromDegrees(180.0))

    //Offset poses for robot geometry (14 inches is the distance from hatch wheels to the wrist pivot)
    val lowHatchWheelsFrontTransform = Pose2d(
        -abs(ControlParameters.SuperstructurePositions.rocketHatchBottomFront.point.x.value) - 14.0,
        0.0,
        Rotation2d.identity()
    )

    val midHatchWheelsFrontTransform = Pose2d(
        -abs(ControlParameters.SuperstructurePositions.rocketHatchMidFront.point.x.value) - 14.0,
        0.0,
        Rotation2d.identity()
    )

    val intakeHatchWheelsBackTransform = Pose2d(
        -abs(ControlParameters.SuperstructurePositions.hatchIntakeBack.point.x.value) - 14.0,
        0.0,
        Rotation2d.identity()
    )

    //Path transforms
    val targetAlignmentTransform = Pose2d(
        -42.0, //Allow 4 feet to align with any target
        0.0,
        Rotation2d.identity()
    )

    val clearanceTransform = Pose2d(
        -12.0, //Allow 1 foot to reverse from any target before turning
        0.0,
        Rotation2d.identity()
    )

    val forwardHeading = Rotation2d.identity()

    //Robot trajectory poses
    val trajRight1 = fieldToNearRocketRight.transformBy(lowHatchWheelsFrontTransform) //Low hatch scoring position
    val trajRight2 = Pose2d.fromTranslation(trajRight1.translation.translateBy(Translation2d(-3.0 * 12.0, 0.0))).withHeading(Rotation2d.fromDegrees(15.0))
    val trajRight3 = fieldToInboundingStationRight.transformBy(targetAlignmentTransform).withHeading(forwardHeading) //Aligning with station
    val trajRight4 = fieldToInboundingStationRight.transformBy(intakeHatchWheelsBackTransform).withHeading(forwardHeading) //Intaking hatch
    val trajRight5 = trajRight4.transformBy(clearanceTransform) //Backed up from station after intaking
    val trajRight6 = fieldToNearRocketRight.transformBy(targetAlignmentTransform) //Aligning with rocket
    val trajRight7 = fieldToNearRocketRight.transformBy(midHatchWheelsFrontTransform) //Scoring on rocket
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
    val level1HabToNearRocketRightWaypoints = listOf(
        CriticalPoses.trajRight4,
        CriticalPoses.trajRight5,
        CriticalPoses.trajRight6,
        CriticalPoses.trajRight7
    )

    println(genJson(level1HabToNearRocketRightWaypoints))
}