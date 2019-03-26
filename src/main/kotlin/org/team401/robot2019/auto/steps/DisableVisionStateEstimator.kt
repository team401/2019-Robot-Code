package org.team401.robot2019.auto.steps

import org.snakeskin.auto.steps.SingleStep
import org.team401.robot2019.control.vision.VisionOdometryUpdater

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 */
class DisableVisionStateEstimator: SingleStep() {
    override fun entry(currentTime: Double) {
        VisionOdometryUpdater.disable()
    }
}