package org.team401.robot2019.control.superstructure.planning.command

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.arm.control.ArmKinematics

/**
 * @author Cameron Earle
 * @version 2/18/2019
 *
 */
class SetWristAngleCommand(val tool: WristMotionPlanner.Tool, val angle: AngularDistanceMeasureRadians, val targetPose: Point2d): SuperstructureCommand() {
    var done = false
    private val targetArmPolar = ArmKinematics.inverse(targetPose)
    private val targetArmState = ArmState(targetArmPolar.r, targetArmPolar.theta, 0.0.RadiansPerSecond)

    override fun entry() {
        WristMotionPlanner.setToAngleMode(tool, angle, targetPose)
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState) {
        val wristCommand = WristMotionPlanner.update(armState, wristState)
        SuperstructureController.update(targetArmState, wristCommand, tool)
        println("Moving wrist from ${wristState.wristPosition.toDegrees()} to ${wristCommand.wristPosition}")
        done = true
    }

    override fun isDone(): Boolean {
        return done
    }

    override fun getDescription(): String {
        return "Target Tool: $tool | Target Floor Relative Angle: $angle | Final Arm Pose: $targetPose"
    }
}