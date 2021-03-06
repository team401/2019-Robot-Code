package org.team401.robot2019.auto.steps.climber

import org.snakeskin.auto.steps.AutoStep
import org.team401.robot2019.subsystems.ClimberSubsystem

/**
 * Tells the climber to home and waits for it to finish
 */
class HomeClimberStep: AutoStep() {
    override fun entry(currentTime: Double) {
        if (!ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.Homing)) {
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.Homing).waitFor()
        }
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        return ClimberSubsystem.homed
    }

    override fun exit(currentTime: Double) {
    }
}