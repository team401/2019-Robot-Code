package org.team401.robot2019.control.drivetrain

import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.Physics
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.robot2019.util.fieldMirror
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
    private val maxVel = 6.0 * 12
    private val maxAccel = 2.0 * 12
    private val maxVoltage = 9.0

    private fun flipWaypoints(waypointsIn: List<Pose2d>): List<Pose2d> {
        val waypoints = ArrayList<Pose2d>(waypointsIn.size)
        waypointsIn.forEach {
            val poseMirrored = it.fieldMirror()
            waypoints.add(poseMirrored)
        }

        return waypoints
    }

    private fun generateTrajectory(points: List<Pose2d>, reversed: Boolean = false, constraints: List<TimingConstraint<Pose2dWithCurvature>> = listOf()): Trajectory<TimedState<Pose2dWithCurvature>> {
        val allConstraints = arrayListOf<TimingConstraint<Pose2dWithCurvature>>(CentripetalAccelerationConstraint(maxCentrip))
        allConstraints.addAll(constraints)

        return pm.generateTrajectory(
            reversed,
            points,
            allConstraints,
            maxVel,
            maxAccel,
            maxVoltage
        )
    }

    /*
    private val level1HabToFarRocketRightWaypoints = listOf(
        CriticalPoses.fieldToLevel1RightStart,
        CriticalPoses.fieldToFarRocketRightMidpoint,
        CriticalPoses.fieldToFarRocketRightAlign,
        CriticalPoses.fieldToFarRocketRightEnd
    )
    */

    val level1HabToNearRocketRightWaypoints = listOf(
        CriticalPoses.fieldToLevel1RightStart,
        CriticalPoses.fieldToNearRocketRightAlign,
        CriticalPoses.fieldToNearRocketRightEnd
    )

    val nearRocketRightToInboundingStationRightWaypoints = listOf(
        CriticalPoses.fieldToNearRocketRightEnd,
        CriticalPoses.fieldToNearRocketRightBackUp,
        CriticalPoses.fieldToInboundingStationRightAlign,
        CriticalPoses.fieldToInboundingStationRightEnd
    )

    val inboundingStationRightToNearRocketRightWaypoints = listOf(
        CriticalPoses.fieldToInboundingStationRightEnd,
        CriticalPoses.fieldToInboundingStationRightBackUp,
        CriticalPoses.fieldToNearRocketRightAlign,
        CriticalPoses.fieldToNearRocketRightEnd
    )

    //private val level1HabToFarRocketLeftWaypoints = flipWaypoints(level1HabToFarRocketRightWaypoints)
    val level1HabToNearRocketLeftWaypoints = flipWaypoints(level1HabToNearRocketRightWaypoints)
    val nearRocketLeftToInboundingStationLeftWaypoints = flipWaypoints(nearRocketRightToInboundingStationRightWaypoints)
    val inboundingStationLeftToNearRocketLeftWaypoints = flipWaypoints(inboundingStationRightToNearRocketRightWaypoints)

    /*
    val level1HabToFarRocketRight = generateTrajectory(
        level1HabToFarRocketRightWaypoints,
        false
    )
    */

    val level1HabToNearRocketRight = generateTrajectory(
        level1HabToNearRocketRightWaypoints,
        false
    )

    /*
    val level1HabToFarRocketLeft = generateTrajectory(
        level1HabToFarRocketLeftWaypoints,
        false
    )
    */

    val level1HabToNearRocketLeft = generateTrajectory(
        level1HabToNearRocketLeftWaypoints,
        false/*,
        listOf(VelocityLimitRegionConstraint(
            Translation2d(0.0, Double.NEGATIVE_INFINITY),
            Translation2d(120.0, Double.POSITIVE_INFINITY),
            2.0 * 12
        ))*/
    )

    val nearRocketRightToInboundingStationRight = generateTrajectory(
        nearRocketRightToInboundingStationRightWaypoints,
        true
    )

    val nearRocketLeftToInboundingStationLeft = generateTrajectory(
        nearRocketLeftToInboundingStationLeftWaypoints,
        true
    )

    val inboundingStationRightToNearRocketRight = generateTrajectory(
        inboundingStationRightToNearRocketRightWaypoints,
        false
    )

    val inboundingStationLeftToNearRocketLeft = generateTrajectory(
        inboundingStationLeftToNearRocketLeftWaypoints,
        false
    )
}