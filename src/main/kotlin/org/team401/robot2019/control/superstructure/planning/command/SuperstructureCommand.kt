package org.team401.robot2019.control.superstructure.planning.command

import org.team401.robot2019.control.superstructure.geometry.ArmState
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
            println("Starting Command:\t${javaClass.simpleName}")
            println("Description:\t${getDescription()}")
            entry()
            isStarted = true
        } else {
            action(dt, armState, wristState)
        }
    }

    abstract fun entry()
    abstract fun action(dt: Double, armState: ArmState, wristState: WristState)
    abstract fun isDone(): Boolean

    /**
     * Returns a description of this specific instance of a command.  This will be called if an exception is
     * encountered while running a command.  It will also be printed if the motion planner is in debug mode
     * when a command is about to be started by the executor.
     *
     * The format of this command should contain all of the parameters of interest
     * during a crash that could be used to recreate the issue in simulation.
     */
    abstract fun getDescription(): String
}