package org.team401.robot2019.control.superstructure.planning.command

import org.snakeskin.hardware.Hardware
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.taxis.geometry.Pose2d

class DelayCommand(val time: TimeMeasureSeconds): SuperstructureCommand() {
    var startTime = 0.0.Seconds

    override fun entry() {
        startTime = Hardware.getRelativeTime().Seconds
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState, drivePose: Pose2d) {

    }

    override fun isDone(): Boolean {
        return (Hardware.getRelativeTime().Seconds - startTime) >= time
    }

    override fun getDescription(): String {
        return "Delay($time)"
    }
}