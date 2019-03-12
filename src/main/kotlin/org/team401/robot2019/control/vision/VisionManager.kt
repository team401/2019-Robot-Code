package org.team401.robot2019.control.vision

import edu.wpi.first.networktables.*
import org.snakeskin.logic.LockingDelegate

object VisionManager {
    val frontCamera = LimelightCamera("limelight-front")
    //val backCamera = LimelightCamera("limelight-back")

    fun start() {
        frontCamera.startListening()
        //backCamera.startListening()
    }
}