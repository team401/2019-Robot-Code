package org.team401.robot2019.auto.steps.superstructure

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SingleStep
import org.team401.robot2019.subsystems.ArmSubsystem

/**
 * Tells the arm that it is homed, and moves it to the safe position
 * Step unblocks when the homed flag is set in the subsystem
 */
class ArmHomeStep: AutoStep() {
    override fun entry(currentTime: Double) {
        if (!ArmSubsystem.extensionHomed) {
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.SetHome)
        }
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        return ArmSubsystem.extensionHomed //We're done when this flag gets set
    }

    override fun exit(currentTime: Double) {
    }
}