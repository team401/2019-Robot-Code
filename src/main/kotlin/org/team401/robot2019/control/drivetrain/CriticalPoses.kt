package org.team401.robot2019.control.drivetrain

import org.team401.robot2019.config.Geometry
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.trajectory.TrajectoryUtil

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 */
object CriticalPoses {
    //Field relative positions of various field elements
    val fieldToNearRocketRight = Pose2d(213.678, 18.116, Rotation2d.fromDegrees(331.23))
    val fieldToFarRocketRight = Pose2d(243.697, 18.116, Rotation2d.fromDegrees(209.77))
    val fieldToRocketCargoRight = Pose2d(229.063, 27.442, Rotation2d.fromDegrees(270.0))
    val fieldToRocketCargoStraightRight = Pose2d.fromTranslation(fieldToRocketCargoRight.translation)
    val fieldToNearRocketLeft = Pose2d(213.678, 303.884, Rotation2d.fromDegrees(29.77))
    val fieldToFarRocketLeft = Pose2d(243.697, 303.884, Rotation2d.fromDegrees(151.23))
    val fieldToInboundingStationRight = Pose2d(0.0, 25.715, Rotation2d.fromDegrees(180.0))
    val fieldToInboundingStationLeft = Pose2d(0.0, 296.285, Rotation2d.fromDegrees(180.0))
    val fieldToLevel2HabCorner = Pose2d(0.0, 137.0, Rotation2d.identity())
    val fieldToLevel1HabCorner = Pose2d(48.0, 137.0, Rotation2d.identity())

    //Robot geometry transforms
    val robotBackLeftToOriginTransform = Pose2d(
        Geometry.DrivetrainGeometry.robotHalfLength.value,
        -Geometry.DrivetrainGeometry.robotHalfWidth.value,
        Rotation2d.identity()
    )

    val robotFrontCenterToOriginTransform = Pose2d(
        -Geometry.DrivetrainGeometry.robotHalfLength.value,
        0.0,
        Rotation2d.identity()
    )

    //Path transforms
    val targetAlignmentTransform = Pose2d(
        -24.0, //Allow 3 feet to align with any target
        0.0,
        Rotation2d.identity()
    )

    //Robot trajectory poses
    val fieldToLevel1RightStart = fieldToLevel1HabCorner.transformBy(robotBackLeftToOriginTransform)!!
    val fieldToLevel2RightStart = fieldToLevel2HabCorner.transformBy(robotBackLeftToOriginTransform)!!

    val fieldToFarRocketRightEnd = fieldToFarRocketRight.transformBy(robotFrontCenterToOriginTransform)!!
    val fieldToNearRocketRightEnd = fieldToNearRocketRight.transformBy(robotFrontCenterToOriginTransform)!!

    val fieldToFarRocketRightAlign = fieldToFarRocketRightEnd.transformBy(targetAlignmentTransform)!!
    val fieldToNearRocketRightAlign = fieldToNearRocketRightEnd.transformBy(targetAlignmentTransform)!!

    val fieldToFarRocketRightMidpoint = fieldToRocketCargoStraightRight.transformBy(Pose2d(0.0, 6.0 * 12, Rotation2d.identity()))!!
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

fun genJson(vararg poses: Pose2d): String {
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

    println(CriticalPoses.fieldToLevel1RightStart)
    println(CriticalPoses.fieldToFarRocketRightMidpoint)
    println(CriticalPoses.fieldToFarRocketRightAlign)
    println(CriticalPoses.fieldToFarRocketRightEnd)

    println(genJson(
        CriticalPoses.fieldToLevel1RightStart,
        CriticalPoses.fieldToFarRocketRightMidpoint,
        CriticalPoses.fieldToFarRocketRightAlign,
        CriticalPoses.fieldToFarRocketRightEnd
    ))
}