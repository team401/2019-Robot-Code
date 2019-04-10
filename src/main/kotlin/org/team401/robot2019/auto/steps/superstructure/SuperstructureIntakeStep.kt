package org.team401.robot2019.auto.steps.superstructure

import org.team401.robot2019.control.superstructure.SuperstructureRoutines

class SuperstructureIntakeStep(val side: SuperstructureRoutines.Side): SuperstructureAutoStep() {
    override fun command(currentTime: Double) {
        SuperstructureRoutines.side = side //Set correct side
        SuperstructureRoutines.intake()
    }
}