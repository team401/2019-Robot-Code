package org.team401.robot2019.control.vision

import org.team401.robot2019.config.Geometry

object VisionManager {
    val frontCamera = LimelightCamera("limelight-front", Geometry.VisionGeometry.robotToFrontCamera)
    val backCamera = LimelightCamera("limelight-back", Geometry.VisionGeometry.robotToBackCamera)

    fun start() {
        frontCamera.startListening()
        backCamera.startListening()
    }
}