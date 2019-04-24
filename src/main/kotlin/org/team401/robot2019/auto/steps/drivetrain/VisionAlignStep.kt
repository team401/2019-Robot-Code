package org.team401.robot2019.auto.steps.drivetrain

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.subsystems.DrivetrainSubsystem

class VisionAlignStep(val side: SuperstructureRoutines.Side, timeout: TimeMeasureSeconds, forwardPower: Double = .2, val pIn: Double = 0.6): AutoStep(){
    lateinit var activeCamera: LimelightCamera
    private val timeoutSeconds = timeout.value
    private val output = when (side) {
        SuperstructureRoutines.Side.FRONT -> forwardPower
        SuperstructureRoutines.Side.BACK -> -forwardPower
    }

    private var hasEverSeenTarget = false
    private var startTime = 0.0

    override fun entry(currentTime: Double) {
        startTime = currentTime

        activeCamera = when (side) {
            SuperstructureRoutines.Side.FRONT -> {
                VisionManager.frontCamera
            }
            SuperstructureRoutines.Side.BACK -> {
                VisionManager.backCamera
            }
        }

        activeCamera.configForVision(3)
        activeCamera.setLedMode(LimelightCamera.LedMode.UsePipeline)
        activeCamera.resetFrame()

        DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ExternalControl).waitFor()
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        val hasTarget = activeCamera.entries.tv.getDouble(0.0) == 1.0
        if (hasTarget) { // If the robot sees the target, continue
            hasEverSeenTarget = true
            //println("I see the target!")
            val activeOffset = ControlParameters.VisionOffsets.select(
                side,
                SuperstructureController.output.visionHeightMode,
                SuperstructureController.output.wristTool
            )
            val tx = activeCamera.entries.tx.getDouble(0.0)
            val txOffset = tx - activeOffset.value
            //println("Active offset: $activeOffset\tHas target: $hasTarget")

            val adjustment = if (hasTarget) {
                (pIn * txOffset) / 100.0 // 1 degree of error = 1%power
            } else {
                0.0
            }

            val leftPower = output + adjustment
            val rightPower = output - adjustment

            DrivetrainSubsystem.tank(leftPower, rightPower)

        } else { // The robot cannot see the camera, so it should be at the rocket
            if (!hasEverSeenTarget) {
                DrivetrainSubsystem.tank(output, output)
            }
        }

        return (currentTime - startTime) >= timeoutSeconds
    }

    override fun exit(currentTime: Double) {
        DrivetrainSubsystem.stop()
    }
}