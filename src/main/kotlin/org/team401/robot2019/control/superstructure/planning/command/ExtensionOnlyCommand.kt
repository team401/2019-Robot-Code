package org.team401.robot2019.control.superstructure.planning.command

import org.snakeskin.measure.Inches
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

/**
 * @author Cameron Earle
 * @version 2/18/2019
 *
 * Clears the extension out to the safe limit.  Done when the extension is at the safe position
 */
class ExtensionOnlyCommand(val radius: LinearDistanceMeasureInches, val tool: WristMotionPlanner.Tool, val tolerance: LinearDistanceMeasureInches = 1.0.Inches): SuperstructureCommand() {
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
        }

        SuperstructureController.update(
            ArmState(
                radius,
                armInitial!!.armAngle,
                0.0.RadiansPerSecond
            ),
            wristInitial!!,
            tool
        )

        done = Math.abs(armState.armRadius.value - radius.value) <= tolerance.value //Allow some error
    }

    override fun isDone(): Boolean {
        return done
    }
}