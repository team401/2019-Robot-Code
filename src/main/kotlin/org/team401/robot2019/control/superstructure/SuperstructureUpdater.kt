package org.team401.robot2019.control.superstructure

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2019.Gamepad
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.subsystems.ArmSubsystem
import org.team401.robot2019.subsystems.WristSubsystem
import org.team401.robot2019.subsystems.arm.control.ArmKinematics

/**
 * @author Cameron Earle
 * @version 2/11/2019
 *
 * RT task to update the superstructure motion planner with the current system state
 */
object SuperstructureUpdater: RealTimeTask {
    override val name = "Superstructure Updater"
    private val ds = DriverStation.getInstance()

    override fun action(ctx: RealTimeExecutor.RealTimeContext) {
        if (ds.isEnabled) {
            SuperstructureMotionPlanner.updateJog(Gamepad.readAxis { LEFT_X }, Gamepad.readAxis { LEFT_Y })
        }
        SuperstructureMotionPlanner.update(
            ctx.time,
            ctx.dt,
            ArmSubsystem.getCurrentArmState(),
            WristSubsystem.getCurrentWristState()
        )

        //debug
        val targetPose = ArmKinematics.forward(PointPolar(SuperstructureController.output.armRadius, SuperstructureController.output.armAngle))
        val actualPose = ArmKinematics.forward(ArmSubsystem.getCurrentArmState())
        SmartDashboard.putNumber("superstructure_x_error_inches", (targetPose.x - actualPose.x).value)
        SmartDashboard.putNumber("superstructure_y_error_inches", (targetPose.y - actualPose.y).value)
    }
}