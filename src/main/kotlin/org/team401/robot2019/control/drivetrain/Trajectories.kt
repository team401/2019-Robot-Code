package org.team401.robot2019.control.drivetrain

import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.Physics
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.robot2019.util.fieldMirror
import org.team401.robot2019.util.withHeading
import org.team401.taxis.diffdrive.control.DrivetrainPathManager
import org.team401.taxis.diffdrive.control.FeedforwardOnlyPathController
import org.team401.taxis.diffdrive.control.FullStateDiffDriveModel
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.TrajectoryUtil
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint
import org.team401.taxis.trajectory.timing.TimedState
import org.team401.taxis.trajectory.timing.TimingConstraint
import org.team401.taxis.trajectory.timing.VelocityLimitRegionConstraint

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 */
object Trajectories {
    private val pm = DrivetrainSubsystem.pathManager
    private val maxCentrip = 110.0
    private val maxVel = 8.0 * 12
    private val maxAccel = 5.0 * 12
    private val maxVoltage = 9.0

    private fun flipWaypoints(waypointsIn: List<Pose2d>): List<Pose2d> {
        val waypoints = ArrayList<Pose2d>(waypointsIn.size)
        waypointsIn.forEach {
            val poseMirrored = it.fieldMirror()
            waypoints.add(poseMirrored)
        }

        return waypoints
    }

    private fun generateTrajectory(points: List<Pose2d>, reversed: Boolean = false, endVelocity: Double = 0.0, constraints: List<TimingConstraint<Pose2dWithCurvature>> = listOf()): Trajectory<TimedState<Pose2dWithCurvature>> {
        val allConstraints = arrayListOf<TimingConstraint<Pose2dWithCurvature>>()
        allConstraints.add(CentripetalAccelerationConstraint(maxCentrip))
        allConstraints.addAll(constraints)

        return pm.generateTrajectory(
            reversed,
            points,
            allConstraints,
            0.0,
            endVelocity,
            maxVel,
            maxAccel,
            maxVoltage
        )
    }

    val rocketRightToHatchIntakeRightWaypoints = listOf(
        CriticalPoses.trajRight1,
        CriticalPoses.trajRight2,
        CriticalPoses.trajRight3,
        CriticalPoses.trajRight4
    )

    val rocketLeftToHatchIntakeLeftWaypoints = flipWaypoints(rocketRightToHatchIntakeRightWaypoints)

    val hatchIntakeRightToRocketRightWaypoints = listOf(
        CriticalPoses.trajRight4,
        CriticalPoses.trajRight5,
        CriticalPoses.trajRight6,
        CriticalPoses.trajRight7
    )

    val hatchIntakeLeftToRocketLeftWaypoints = flipWaypoints(hatchIntakeRightToRocketRightWaypoints)

    val rocketRightToHatchIntakeRight = generateTrajectory(
        rocketRightToHatchIntakeRightWaypoints,
        true
    )

    val rocketLeftToHatchIntakeLeft = generateTrajectory(
        rocketLeftToHatchIntakeLeftWaypoints,
        true
    )

    val hatchIntakeRightToRocketRight = generateTrajectory(
        hatchIntakeRightToRocketRightWaypoints,
        false
    )

    val hatchIntakeLeftToRocketLeft = generateTrajectory(
        hatchIntakeLeftToRocketLeftWaypoints,
        false
    )
}