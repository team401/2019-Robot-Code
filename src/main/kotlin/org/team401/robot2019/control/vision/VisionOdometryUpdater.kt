package org.team401.robot2019.control.vision

import org.snakeskin.logic.LockingDelegate
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.taxis.diffdrive.odometry.DifferentialDriveState
import org.team401.taxis.geometry.Pose2d

object VisionOdometryUpdater: RealTimeTask {
    private val driveState: DifferentialDriveState = DrivetrainSubsystem.driveState
    private var lastFrameCaptureTime = 0.0

    override val name = "Vision Odometry Updater"
    private var enabled by LockingDelegate(false)

    private var fieldToGoal by LockingDelegate(Pose2d.identity())
    private var activeCamera: LimelightCamera by LockingDelegate(VisionManager.frontCamera)

    @Synchronized fun enable(fieldToGoal: Pose2d, camera: LimelightCamera) {
        this.fieldToGoal = fieldToGoal
        this.activeCamera = camera
        lastFrameCaptureTime = 0.0
        activeCamera.configForVision(1)
        activeCamera.setLedMode(LimelightCamera.LedMode.UsePipeline)
        activeCamera.resetFrame()
        enabled = true
    }

    @Synchronized fun disable() {
        activeCamera.configForVision(1)
        activeCamera.setLedMode(LimelightCamera.LedMode.Off)
        enabled = false
    }


    override fun action(ctx: RealTimeExecutor.RealTimeContext) {
        val frame = activeCamera.frame
        if (enabled && frame.hasTarget) {
            if (frame.timeCaptured.value > lastFrameCaptureTime) {
                //println("x: ${frame.x} y: ${frame.y} yaw: ${frame.yaw}")
                //We got a new frame, let's process it
                //Create a pose from the camera data
                val goalToCamera = frame.toPose2d()
                //Run kinematics.  This is the actual pose at the time of capture
                val observedFieldToRobot = VisionKinematics.solveFieldToRobot(fieldToGoal, activeCamera.robotToCamera, goalToCamera)
                //Store this observation in the state tracker
                VisionState.addVisionObservation(frame.timeCaptured.value, observedFieldToRobot)
                //Update this so we don't process this frame again
                lastFrameCaptureTime = frame.timeCaptured.value
            }
        }
    }
}