package org.team401.robot2019.auto.steps.superstructure

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SingleStep
import org.team401.robot2019.subsystems.ArmSubsystem

/**
 * Tells the arm that it is homed, and moves it to the safe position
 * Step unblocks when the homed flag is set in the subsystem
 */
class ArmHomeStep: SingleStep() {
    override fun entry(currentTime: Double) {
        if (!ArmSubsystem.extensionHomed) {
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.SetHome).waitFor()
        }
    }
}