package org.team401.robot2019.control.drivetrain

import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 */
object CriticalPoses {
    val robotHalfPose = Pose2d(17.5, 0.0, Rotation2d.identity())
    val distanceFromGoal = Pose2d(6.0, 0.0, Rotation2d.identity())
    val totalFromGoalOffset = robotHalfPose.inverse().transformBy(distanceFromGoal.inverse())

    //Distance from target to align with it
    val alignPose = Pose2d(3.0 * 12, 0.0, Rotation2d.identity())

    val originToEndOfLevel2 = Pose2d(48.0, 0.0, Rotation2d.identity())
    val originToRobotCenterY = Pose2d(0.0, 207.0, Rotation2d.identity())
    val robotStartLeft = originToEndOfLevel2.transformBy(robotHalfPose).transformBy(originToRobotCenterY)

    val leftRocketFarGoal = Pose2d(243.759, 303.884, Rotation2d.fromDegrees(151.23))
    val leftRocketNearGoal = Pose2d(214.491, 303.884, Rotation2d.fromDegrees(28.77))
    val leftFeederStationGoal = Pose2d(0.0, 296.285, Rotation2d.fromDegrees(180.0))

    val leftRocketFarScoring = leftRocketFarGoal.transformBy(totalFromGoalOffset)
    val leftRocketFarAlign = leftRocketFarScoring.transformBy(alignPose.inverse())
}