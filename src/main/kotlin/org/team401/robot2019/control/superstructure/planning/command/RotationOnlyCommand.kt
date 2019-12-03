package org.team401.robot2019.control.superstructure.planning.command

import org.snakeskin.measure.Inches
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.control.superstructure.geometry.VisionHeightMode
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.ArmMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.taxis.geometry.Pose2d

/**
 * @author Cameron Earle
 * @version 2/19/2019
 *
 */
class RotationOnlyCommand(val angle: AngularDistanceMeasureRadians, val tool: WristMotionPlanner.Tool): SuperstructureCommand() {
    var done = false
    var wristInitial: WristState? = null
    var armInitial: ArmState? = null

    override fun entry() {
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState, drivePose: Pose2d) {
        if (wristInitial == null) {
            wristInitial = wristState
        }
        if (armInitial == null) {
            armInitial = armState
            ArmMotionPlanner.setDesiredTrajectory(
                ArmKinematics.forward(PointPolar(armInitial!!.armRadius, armInitial!!.armAngle)),
                ArmKinematics.forward(PointPolar(armInitial!!.armRadius, angle)),
                armInitial!!.armRadius - 0.1.Inches
            )
        }
        val armCommand = ArmMotionPlanner.update(dt, drivePose)

        SuperstructureController.update(
            ArmState(
                armInitial!!.armRadius,
                armCommand.armAngle,
                armCommand.armVelocity
            ),
            wristInitial!!,
            tool,
            VisionHeightMode.NONE
        )

        done = ArmMotionPlanner.isDone()
        if (done) {
            //Push a last command through with the final state
            val finalArmState = ArmState(armInitial!!.armRadius, angle, 0.0.RadiansPerSecond)
            SuperstructureController.update(finalArmState, wristInitial!!, tool, VisionHeightMode.NONE)
        }
    }

    override fun isDone(): Boolean {
        return done
    }

    override fun getDescription(): String {
        return "Target Arm Angle: $angle | Active Tool: $tool"
    }
}