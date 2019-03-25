package org.team401.robot2019.control.vision

import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.util.InterpolatingDouble
import org.team401.taxis.util.InterpolatingTreeMap

/**
 * Wrapper for the drivetrain's DifferentialDriveState, adjusts data based on inputs from the vision system.
 * The VisionOdometryUpdater adds observations from either the front or the back camera, which
 * can then be used to retrieve the measured robot pose at a given timestamp, interpolated linearly
 * where applicable
 */
object VisionState {
    val identity = InterpolatingDouble(Double.NaN) to Pose2d.identity()

    /**
     * Observations are stored as the measured field to robot frame at the time of capture.
     */
    private val observations = InterpolatingTreeMap<InterpolatingDouble, Pose2d>(100)

    private val driveState = DrivetrainSubsystem.driveState

    /**
     * Clears the vision observation buffer
     */
    @Synchronized fun reset() {
        observations.clear()
    }

    /**
     * Gets the latest observation from the vision system, or the identity state if there are no observations
     */
    @Synchronized fun getLatestObservation(): Pair<InterpolatingDouble, Pose2d> {
        if (observations.isEmpty()) {
            return identity
        }
        return observations.lastEntry().toPair()
    }

    /**
     * Adds an observation from a vision camera to the system
     */
    @Synchronized fun addVisionObservation(captureTime: Double, observedFieldToRobot: Pose2d) {
        observations[InterpolatingDouble(captureTime)] = observedFieldToRobot
    }

    /**
     * Gets the field to robot pose, using the latest available frame from the camera
     * combined with odometry data from the drive to produce a latency compensated estimate
     * of the robot's pose.
     */
    @Synchronized fun getFieldToRobot(timestamp: Double): Pose2d {
        //Unpack data
        val odometryAtTimestamp = driveState.getFieldToVehicle(timestamp)
        val latestObservation = getLatestObservation()
        val frameCaptureTimestamp = latestObservation.first.value
        val frameCapturePose = latestObservation.second
        val odometryAtCapture = driveState.getFieldToVehicle(frameCaptureTimestamp)

        if (frameCaptureTimestamp.isNaN()) {
            //There is no vision data, just return the odometry field to robot
            return odometryAtTimestamp
        }

        //Do latency correction
        val odometryDisplacement = odometryAtCapture.inverse().transformBy(odometryAtTimestamp)

        return frameCapturePose.transformBy(odometryDisplacement)
    }

    /**
     * Invokes getFieldToRobot with the latest timestamp from the drivetrain's state estimator
     */
    @Synchronized fun getLatestFieldToRobot(): Pose2d {
        val latestTime = driveState.getLatestFieldToVehicle().key.value
        return getFieldToRobot(latestTime)
    }

}