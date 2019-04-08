package org.team401.robot2019.control.drivetrain

import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.Physics
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.FeedforwardOnlyPathController
import org.team401.taxis.diffdrive.control.FullStateDiffDriveModel
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.TrajectoryUtil
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

    private fun flipWaypoints(waypointsIn: List<Pose2d>): List<Pose2d> {
        val waypoints = ArrayList<Pose2d>(waypointsIn.size)
        waypointsIn.forEach {
            val poseMirrored = Pose2d(it.translation.x(), 324.0 - it.translation.y(), Rotation2d(it.rotation.inverse()))
            waypoints.add(poseMirrored)
        }

        return waypoints
    }

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

    private val level1HabToFarRocketRightWaypoints = listOf(
        CriticalPoses.fieldToLevel1RightStart,
        CriticalPoses.fieldToFarRocketRightMidpoint,
        CriticalPoses.fieldToFarRocketRightAlign,
        CriticalPoses.fieldToFarRocketRightEnd
    )

    private val level1HabToFarRocketLeftWaypoints = flipWaypoints(level1HabToFarRocketRightWaypoints)

    val level1HabToFarRocketRight = generateTrajectory(
        level1HabToFarRocketRightWaypoints,
        false
    )

    val level1HabToFarRocketLeft = generateTrajectory(
        level1HabToFarRocketLeftWaypoints,
        false
    )
}