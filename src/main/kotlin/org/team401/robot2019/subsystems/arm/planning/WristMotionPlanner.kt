package org.team401.robot2019.subsystems.arm.planning

import org.snakeskin.measure.Radians
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.subsystems.arm.geometry.ArmState
import org.team401.robot2019.subsystems.arm.geometry.WristState

object WristMotionPlanner {
    //Tools on the wrist
    enum class Tool {
        CargoTool,
        HatchPanelTool
    }

    private val minTopAngle = (Math.PI / 6.0).Radians
    private val maxTopAngle = (2 * Math.PI / 6.0).Radians

    /*
    0 is cargo intake is active tool, parallel to the ground, and at the front of the robot
    PI is the cargo intake is active tool and parallel to the ground on the opposite side
    thus..
    0 is also when the hatch is active, parallel to the ground and the rear side of the robot
    PI is when the hatch is active, parallel to the ground and at the front of the robot
     */

    fun update(commandedArmState: ArmState, currentWristState: WristState): WristState {
        val armAngle = commandedArmState.armAngle
        val armRadius = commandedArmState.armRadius
        var wristAngle = 0.0.Radians
        if (armRadius < Geometry.ArmGeometry.minSafeWristRotation) { // Should always be true
            //throw InvalidPointException("The wrist is not safe to rotate")
        }
        when {
            armAngle > minTopAngle && armAngle < maxTopAngle -> {
                if (currentWristState.currentTool == Tool.CargoTool){
                    wristAngle = (Math.PI / 2.0).Radians // TODO Find what this is all about
                }else{
                    wristAngle = (3 * Math.PI /2.0).Radians // Should move the correct direction. Negate if not?
                }
            }
            armAngle > (Math.PI / 2.0).Radians -> {wristAngle = Math.PI.Radians}
            armAngle < (Math.PI / 2.0).Radians -> {wristAngle = 0.0.Radians}
        }
        return WristState(
            wristAngle,
            currentWristState.currentTool,
            currentWristState.hasCargo,
            currentWristState.hasHatchPanel
        )
    }
}