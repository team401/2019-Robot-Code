package org.team401.robot2019.control.superstructure

import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2019.Gamepad
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.subsystems.ArmSubsystem
import org.team401.robot2019.subsystems.WristSubsystem

/**
 * @author Cameron Earle
 * @version 2/11/2019
 *
 * RT task to update the superstructure motion planner with the current system state
 */
object SuperstructureUpdater: RealTimeTask {
    override val name = "Superstructure Updater"

    override fun action(ctx: RealTimeExecutor.RealTimeContext) {
        SuperstructureMotionPlanner.updateJog(Gamepad.readAxis { LEFT_X }, Gamepad.readAxis { LEFT_Y })
        SuperstructureMotionPlanner.update(
            ctx.time,
            ctx.dt,
            ArmSubsystem.getCurrentArmState(),
            WristSubsystem.getCurrentWristState()
        )
    }
}