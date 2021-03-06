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
import org.team401.robot2019.util.LEDManager

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
            SuperstructureMotionPlanner.updateArmJog(Gamepad.readAxis { LEFT_X }, Gamepad.readAxis { LEFT_Y })
            SuperstructureMotionPlanner.updateWristJog(Gamepad.readAxis { RIGHT_X })
        }

        SuperstructureMotionPlanner.update(
            ctx.time,
            ctx.dt,
            ArmSubsystem.getCurrentArmState(),
            WristSubsystem.getCurrentWristState()
        )

        if (ds.isEnabled) {
            LEDManager.updateToolStatus(SuperstructureController.output.wristTool)
        }

        //debug
        /*
        val targetPose = ArmKinematics.forward(PointPolar(SuperstructureController.output.armRadius, SuperstructureController.output.armAngle))
        val actualState = ArmSubsystem.getCurrentArmState()
        val actualPose = ArmKinematics.forward(actualState)
        SmartDashboard.putNumber("superstructure_x_error_inches", (targetPose.x - actualPose.x).value)
        SmartDashboard.putNumber("superstructure_y_error_inches", (targetPose.y - actualPose.y).value)
        SmartDashboard.putNumber("superstructure_theta_error_degrees", (SuperstructureController.output.armAngle - actualState.armAngle).toDegrees().value)
        SmartDashboard.putNumber("superstructure_radius_error_inches", (SuperstructureController.output.armRadius - actualState.armRadius).toInches().value)
        println("Target Pose : ${targetPose} Wrist: ${WristSubsystem.getCurrentWristState().wristPosition}")
        */


    }
}