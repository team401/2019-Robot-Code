package org.team401.robot2019.control.vision

import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.Seconds
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

object VisionOdometryUpdater: RealTimeTask {
    private val driveState: DifferentialDriveState = DrivetrainSubsystem.driveState
    private var lastFrameCaptureTime = 0.0.Seconds

    override val name = "Vision Odometry Updater"
    private var enabled by LockingDelegate(false)

    private var fieldToGoal by LockingDelegate(Pose2d.identity())
    private var robotToCamera by LockingDelegate(Pose2d.identity())

    @Synchronized fun enable(fieldToGoal: Pose2d, robotToCamera: Pose2d) {
        this.fieldToGoal = fieldToGoal
        this.robotToCamera = robotToCamera
        enabled = true
    }

    @Synchronized fun disable() {
        enabled = false
    }


    override fun action(ctx: RealTimeExecutor.RealTimeContext) {
        val frame = VisionManager.backCamera.frame
        if (enabled && frame.hasTarget) {
            if (frame.timeCaptured > lastFrameCaptureTime) {
                //We got a new frame, let's process it
                //Get the current pose
                val currentPose = driveState.getFieldToVehicle(ctx.time)
                //Get pose at the time the frame was captured
                val poseAtCapture = driveState.getFieldToVehicle(frame.timeCaptured.value)
                //Create a pose from the camera data
                val cameraToGoal = Pose2d(frame.z.value, -frame.x.value, Rotation2d.fromDegrees(frame.yaw.value))
                //Run kinematics.  This is the actual pose at the time of capture
                val measuredPose = VisionKinematics.solveFieldToRobot(poseAtCapture, fieldToGoal, robotToCamera, cameraToGoal)
                //Displacement due to latency
                val displacement = poseAtCapture.inverse().transformBy(currentPose)
                //Transform measured pose by displacement
                val finalPose = measuredPose.transformBy(displacement)
                DrivetrainSubsystem.driveState.addFieldToVehicleObservation(ctx.time, finalPose)
            }
        }
    }
}