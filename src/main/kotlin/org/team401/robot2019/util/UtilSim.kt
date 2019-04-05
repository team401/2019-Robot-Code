package org.team401.robot2019.util

import org.snakeskin.measure.InchesPerSecond
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

fun main(args: Array<String>){

    val startVelocity = 0.0.InchesPerSecond

    val startPose = Pose2d(0.0, 13.0, Rotation2d())
    val straightPart = Pose2d(13.0, 13.0, Rotation2d())
    val goal = Pose2d(15.0, 10.0, Rotation2d())

    /*
    val trajectory = TrajectoryIterator(
        TimedView(
            DrivetrainSubsystem.pathManager.generateTrajectory(
                false,
                listOf(startPose, straightPart, goal),
                listOf(),
                startVelocity.value,
                0.0,
                2.0 * 12.0,
                2.0 * 12.0,
                9.0
            )
        )
    )
    */

    PathReader.outputToPathViewer(TrajectoryPath(false,
        listOf(startPose, straightPart, goal),
        2.0 * 12.0,
        2.0 * 12.0,
        9.0
        ),
        "test_export"
    )




}