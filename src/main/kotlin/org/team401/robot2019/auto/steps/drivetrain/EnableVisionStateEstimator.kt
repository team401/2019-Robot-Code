package org.team401.robot2019.auto.steps.drivetrain

import org.snakeskin.auto.steps.SingleStep
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.control.vision.VisionOdometryUpdater
import org.team401.robot2019.control.vision.VisionState
import org.team401.taxis.geometry.Pose2d

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 */
class EnableVisionStateEstimator(val fieldToGoal: Pose2d, val front: Boolean): SingleStep() {
    override fun entry(currentTime: Double) {
        val camera = if (front) {
            VisionManager.frontCamera
        } else {
            VisionManager.backCamera
        }

        VisionState.reset()
        VisionOdometryUpdater.enable(fieldToGoal, camera)
    }
}