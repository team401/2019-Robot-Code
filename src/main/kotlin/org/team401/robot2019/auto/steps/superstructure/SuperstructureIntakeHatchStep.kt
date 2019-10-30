package org.team401.robot2019.auto.steps.superstructure

import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.subsystems.WristSubsystem

class SuperstructureIntakeHatchStep(): SuperstructureAutoStep() {
    override fun command(currentTime: Double) {
        WristSubsystem.wheelsMachine.setState(WristSubsystem.WristWheelsStates.Intake)
    }
}