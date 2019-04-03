package org.team401.robot2019.auto.steps

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.logic.History

/**
 * Blocks execution of the current sequence when a global "stop" routine is called.
 * This allows you to add spots of "driver intervention" on a button push, allowing
 * the sequence to be resumed when done.
 *
 * Accepts a child step that will be made interruptable by this step
 *
 * Care should be taken that any step that is used with this wrapper can handle potentially long pauses in loop call times.
 * The callbacks can be used to help with this by deconfiguring and reconfiguring anything.
 *
 * Callbacks will always be called in the action state before the child step itself.
 */
class InterruptableAutoStep(private val step: AutoStep, private val onInterrupt: () -> Unit = {}, private val onResume: () -> Unit = {}): AutoStep() {
    enum class InterruptionState {
        RUNNING,
        BLOCK
    }

    companion object {
        private var globalActiveInterventionState = InterruptionState.RUNNING
        @Synchronized get

        /**
         * Blocks all active interruptable steps that are running
         */
        @Synchronized fun interrupt() {
            globalActiveInterventionState = InterruptionState.BLOCK
        }

        /**
         * Resumes all active interruptable steps that are currently blocked
         */
        @Synchronized fun resume() {
            globalActiveInterventionState = InterruptionState.RUNNING
        }
    }

    private val stateHistory = History<InterruptionState>()

    override fun entry(currentTime: Double) {
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        stateHistory.update(globalActiveInterventionState)
        if (stateHistory.current == InterruptionState.BLOCK && stateHistory.last == InterruptionState.RUNNING) {
            //We've just interrupted, call the handler
            onInterrupt()
        }
        if (stateHistory.current == InterruptionState.RUNNING && stateHistory.last == InterruptionState.BLOCK) {
            //We've just resumed, call the handler
            onResume()
        }
        if (globalActiveInterventionState == InterruptionState.RUNNING) {
            step.tick(currentTime, lastTime)
        }
        return step.doContinue()
    }

    override fun exit(currentTime: Double) {
    }
}