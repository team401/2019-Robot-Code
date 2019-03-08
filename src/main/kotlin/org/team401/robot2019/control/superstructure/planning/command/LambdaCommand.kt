package org.team401.robot2019.control.superstructure.planning.command

import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.WristState

/**
 * @author Cameron Earle
 * @version 2/18/2019
 *
 */
class LambdaCommand(val lambda: () -> Unit): SuperstructureCommand() {
    override fun entry() {
        lambda()
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState) {

    }

    override fun isDone(): Boolean {
        return true
    }

    override fun getDescription(): String {
        return "Functional Command"
    }
}