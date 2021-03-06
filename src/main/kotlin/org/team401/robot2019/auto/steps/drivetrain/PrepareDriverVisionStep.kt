package org.team401.robot2019.auto.steps.drivetrain

import org.snakeskin.auto.steps.SingleStep
import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.robot2019.control.vision.VisionManager

class PrepareDriverVisionStep: SingleStep() {
    override fun entry(currentTime: Double) {
        VisionManager.frontCamera.configForVision(0)
        VisionManager.backCamera.configForVision(0)
        VisionManager.frontCamera.setLedMode(LimelightCamera.LedMode.Off)
        VisionManager.backCamera.setLedMode(LimelightCamera.LedMode.Off)
    }
}