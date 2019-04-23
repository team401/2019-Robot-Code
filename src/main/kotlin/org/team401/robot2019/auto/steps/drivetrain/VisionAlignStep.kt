package org.team401.robot2019.auto.steps.drivetrain

import org.snakeskin.auto.steps.AutoStep
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.subsystems.DrivetrainSubsystem

class VisionAlignStep(): AutoStep(){
    lateinit var activeSide: SuperstructureRoutines.Side
    lateinit var activeCamera: LimelightCamera
    private val output = 0.1 // always drives forward at a constant power

    private var counter = 0

    override fun entry(currentTime: Double) {
        activeSide = SuperstructureRoutines.side
        activeCamera = when (activeSide) {
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
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        if (activeCamera.frame.hasTarget) { // If the robot sees the target, continue
            counter = 0
            println("I see the target!")
            val activeOffset = ControlParameters.VisionOffsets.select(
                activeSide,
                SuperstructureController.output.visionHeightMode,
                SuperstructureController.output.wristTool
            )
            val tx = activeCamera.entries.tx.getDouble(0.0)
            val hasTarget = activeCamera.entries.tv.getDouble(0.0) == 1.0
            val txOffset = tx - activeOffset.value
            //println("Active offset: $activeOffset\tHas target: $hasTarget")

            val adjustment = if (hasTarget) {
                (ControlParameters.DrivetrainParameters.visionKp * txOffset) / 100.0 // 1 degree of error = 1%power
            } else {
                0.0
            }

            val leftPower = output + adjustment
            val rightPower = output - adjustment

            DrivetrainSubsystem.tank(leftPower, rightPower)

            return false

        }else{ // The robot cannot see the camera, so it should be at the rocket
            counter++
            return counter >= 20
        }
    }

    override fun exit(currentTime: Double) {
        DrivetrainSubsystem.stop()
    }
}