package org.team401.robot2019.auto.steps.superstructure

import org.team401.robot2019.control.superstructure.SuperstructureRoutines

class SuperstructureStopIntakingStep(): SuperstructureAutoStep() {
    override fun command(currentTime: Double) {
        SuperstructureRoutines.stopIntake()
    }
}