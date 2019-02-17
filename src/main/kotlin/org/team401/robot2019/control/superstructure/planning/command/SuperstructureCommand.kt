package org.team401.robot2019.control.superstructure.planning.command

import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.WristState

/**
 * @author Cameron Earle
 * @version 2/12/2019
 *
 * Represents a generic command for the superstructure to follow
 */
abstract class SuperstructureCommand() {
    private var isStarted = false

    fun update(dt: Double, armState: ArmState, wristState: WristState) {
        if (!isStarted) {
            entry()
            isStarted = true
        } else {
            action(dt, armState, wristState)
        }
    }

    abstract fun entry()
    abstract fun action(dt: Double, armState: ArmState, wristState: WristState)
    abstract fun isDone(): Boolean
}