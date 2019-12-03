package org.team401.robot2019.control.superstructure.planning.command

import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.taxis.geometry.Pose2d

/**
 * @author Cameron Earle
 * @version 2/12/2019
 *
 * Executes a series of commands in parallel (i.e. in "sequence" but each one is updated every cycle)
 */
class ParallelCommand(vararg val sequence: SuperstructureCommand): SuperstructureCommand() {
    override fun entry() {
        sequence.forEach {
            it.entry()
        }
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState, drivePose: Pose2d) {
        sequence.forEach {
            if (!it.isDone()) it.update(dt, armState, wristState, drivePose)
        }
    }

    override fun isDone(): Boolean {
        return sequence.all { it.isDone() }
    }

    override fun getDescription(): String {
        return "Parallel[n]:${sequence.joinToString { it.getDescription() }}"
    }
}