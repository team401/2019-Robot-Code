package org.team401.robot2019.auto.steps.superstructure

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.dsl.on
import org.snakeskin.logic.LockingDelegate
import org.team401.robot2019.RobotEvents

class WaitForHatchAcquiredStep: AutoStep() {
    private var flag by LockingDelegate(false)

    override fun entry(currentTime: Double) {
        on(RobotEvents.HatchAcquired) {
            flag = true
        }
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        return flag
    }

    override fun exit(currentTime: Double) {}
}