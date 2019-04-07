package org.team401.robot2019.auto.steps.superstructure

import org.snakeskin.auto.steps.AutoStep
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner

/**
 * Base class for any autonomous control of the superstructure
 */
abstract class SuperstructureAutoStep: AutoStep() {
    override fun entry(currentTime: Double) {
        command(currentTime)
        SuperstructureRoutines.ccMaybe(true)
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        return SuperstructureMotionPlanner.isDone()
    }

    override fun exit(currentTime: Double) {
    }

    /**
     * The command to do on the superstructure.  This is effectively the entry method
     */
    abstract fun command(currentTime: Double)
}