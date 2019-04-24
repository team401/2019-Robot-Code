package org.team401.robot2019.auto.steps.drivetrain

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2019.subsystems.DrivetrainSubsystem

class OpenLoopReverseStep(val power: Double, val time: TimeMeasureSeconds): AutoStep() {
    private var startTime = 0.0

    override fun entry(currentTime: Double) {
        startTime = currentTime
        DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ExternalControl).waitFor()
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        DrivetrainSubsystem.tank(-power, -power)
        return (currentTime - startTime) >= time.value
    }

    override fun exit(currentTime: Double) {
        DrivetrainSubsystem.stop()
    }
}