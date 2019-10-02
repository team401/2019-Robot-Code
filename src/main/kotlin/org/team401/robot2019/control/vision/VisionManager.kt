package org.team401.robot2019.control.vision

import org.team401.robot2019.config.Geometry
import org.team401.robot2019.vision2.LimelightCameraEnhanced
import org.team401.taxis.geometry.Rotation2d

object VisionManager {
    val frontCamera = LimelightCameraEnhanced("limelight-front", Geometry.VisionGeometry.robotToFrontCamera, Rotation2d.identity(), 32.0)
    val backCamera = LimelightCameraEnhanced("limelight-back", Geometry.VisionGeometry.robotToBackCamera, Rotation2d.identity(), 32.0)

    fun start() {
        frontCamera.startListening()
        backCamera.startListening()
    }

    fun stop() {
        frontCamera.stopListening()
        backCamera.stopListening()
    }
}