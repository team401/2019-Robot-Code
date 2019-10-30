package org.team401.robot2019.auto.steps.superstructure

import org.snakeskin.auto.steps.AutoStep
import org.team401.robot2019.Gamepad

class WaitForScoreButtonStep: AutoStep() {
    override fun entry(currentTime: Double) {}

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        return Gamepad.readButton { RIGHT_BUMPER }
    }

    override fun exit(currentTime: Double) {}
}