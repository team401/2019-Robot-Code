package org.team401.robot2019.control.superstructure.planning.command

import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.WristState

/**
 * @author Cameron Earle
 * @version 2/12/2019
 *
 * Executes a series of commands in sequence
 */
class SeriesCommand(vararg val sequence: SuperstructureCommand): SuperstructureCommand() {
    private var seqIdx = 0

    override fun entry() {
        //Do nothing
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState) {
        sequence[seqIdx].update(dt, armState, wristState)
        if (sequence[seqIdx].isDone()) seqIdx++
    }

    override fun isDone(): Boolean {
        return sequence.last().isDone() //We're done when the last item in the sequence is done
    }
}