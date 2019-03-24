package org.team401.robot2019.control.superstructure.planning.command

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.ArmMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.arm.control.ArmKinematics

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

    override fun action(dt: Double, armState: ArmState, wristState: WristState) {
        if (wristInitial == null) {
            wristInitial = wristState
        }
        if (armInitial == null) {
            armInitial = armState
            ArmMotionPlanner.setDesiredTrajectory(
                ArmKinematics.forward(PointPolar(1.0.Inches, armInitial!!.armAngle)),
                ArmKinematics.forward(PointPolar(1.0.Inches, angle)),
                0.9.Inches
            )
        }
        val armCommand = ArmMotionPlanner.update(dt)

        SuperstructureController.update(
            ArmState(
                armInitial!!.armRadius,
                armCommand.armAngle,
                armCommand.armVelocity
            ),
            wristInitial!!,
            tool
        )
    }

    override fun isDone(): Boolean {
        return ArmMotionPlanner.isDone()
    }

    override fun getDescription(): String {
        return "Target Arm Angle: $angle | Active Tool: $tool"
    }
}