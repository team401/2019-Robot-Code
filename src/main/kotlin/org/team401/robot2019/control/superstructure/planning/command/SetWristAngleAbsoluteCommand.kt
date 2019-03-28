package org.team401.robot2019.control.superstructure.planning.command

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.VisionHeightMode
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

class SetWristAngleAbsoluteCommand(val tool: WristMotionPlanner.Tool, val angle: AngularDistanceMeasureRadians): SuperstructureCommand() {
    var done = false
    private var startArmState: ArmState? = null

    override fun entry() {
        WristMotionPlanner.setToAbsoluteAngleMode(angle)
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState) {
        if (startArmState == null) {
            startArmState = armState
        }
        val wristCommand = WristMotionPlanner.update(armState, wristState)
        SuperstructureController.update(startArmState!!, wristCommand, tool, VisionHeightMode.NONE)
        done = true
    }

    override fun isDone(): Boolean {
        return done
    }

    override fun getDescription(): String {
        return "Target Angle : $angle"
    }
}