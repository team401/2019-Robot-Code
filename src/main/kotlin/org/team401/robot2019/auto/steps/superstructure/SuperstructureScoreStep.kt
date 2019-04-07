package org.team401.robot2019.auto.steps.superstructure

import org.team401.robot2019.control.superstructure.SuperstructureRoutines

/**
 * Scores with whatever the active tool is
 */
class SuperstructureScoreStep: SuperstructureAutoStep() {
    override fun command(currentTime: Double) {
        SuperstructureRoutines.score()
    }
}