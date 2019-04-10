package org.team401.robot2019.auto.steps.drivetrain

import org.snakeskin.auto.steps.SingleStep
import org.team401.robot2019.subsystems.DrivetrainSubsystem

/**
 * Sets the drivetrain to operator control mode
 */
class OperatorDriveStep: SingleStep() {
    override fun entry(currentTime: Double) {
        DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.OpenLoopOperatorControl).waitFor()
    }
}