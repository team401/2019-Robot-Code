package org.team401.robot2019.control.drivetrain

import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint
import org.team401.taxis.trajectory.timing.TimedState

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 */
object Trajectories {
    private val pm = DrivetrainSubsystem.pathManager
    private val maxCentrip = 110.0
    private val maxVel = 8.0 * 12
    private val maxAccel = 8.0 * 12
    private val maxVoltage = 9.0

    private fun generateTrajectory(points: List<Pose2d>, reversed: Boolean = false): Trajectory<TimedState<Pose2dWithCurvature>> {
        return pm.generateTrajectory(
            reversed,
            points,
            listOf(CentripetalAccelerationConstraint(110.0)),
            maxVel,
            maxAccel,
            maxVoltage
        )
    }

    val habLeftLowToFarRocketHighLeft = generateTrajectory(
        listOf(
            CriticalPoses.robotStartLeft,
            CriticalPoses.robotStartLeft.transformBy(Pose2d(18.0 * 12, 3.0 * 12, Rotation2d.identity())),
            CriticalPoses.leftRocketFarAlign,
            CriticalPoses.leftRocketFarScoring
        ),
        false
    )

    /*
    val farRocketHighLeftToFeederStationLeft = generateTrajectory(
        listOf(),
        true
    )

    val feederStationLeftToNearRocketLeft = generateTrajectory(
        listOf(),
        false
    )
    */
}