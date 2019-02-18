package org.team401.robot2019.control.superstructure

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

object SuperstructureController {
    var output by
        LockingDelegate(SuperstructureControlOutput(0.0.Inches, 0.0.Radians, 0.0.Radians, WristMotionPlanner.Tool.CargoTool, 0.0.RadiansPerSecond, 0.0))
    private set

    private fun calculateCounterbalanceVoltage(commandedArmState: ArmState): Double {
        val radius = commandedArmState.armRadius.value
        val theta = commandedArmState.armAngle.value

        //Equation: kS * r * theta
        //Constant kS: Voltage required to hold the superstructure static, divided by r * theta that the test was performed at
        return ControlParameters.ArmParameters.kS * radius * Math.cos(theta)
    }

    private fun calculateVelocityVoltage(commandedArmState: ArmState): Double {
        val velocity = commandedArmState.armVelocity.value

        val dashKv = SmartDashboard.getNumber("ArmKv", 0.0)

        //Equation: kV * desiredVelocity
        //Constant kV: Voltage to velocity relationship (measured after kS
        //return ControlParameters.ArmParameters.kV * velocity
        println("VOLTAGE FROM VELOCITY ${dashKv * velocity}")
        return dashKv * velocity
    }

    /**
     * @param commandedArmState The desired state of the superstructure
     * @param commandedWristState The desired state of the wrist
     *
     * Updates the field "output" with the current control command
     */
    fun update(commandedArmState: ArmState, commandedWristState: WristState, commandedWristTool: WristMotionPlanner.Tool) {
        //Calculate dynamics "counterbalance voltage"
        val counterbalanceFf =
            calculateCounterbalanceVoltage(commandedArmState)

        //Calculate velocity feedforward
        val velocityFf =
            calculateVelocityVoltage(commandedArmState)

        //Calculate total feedforward
        val ff = counterbalanceFf + velocityFf

        //Update control output
        output =
                SuperstructureControlOutput(
                    commandedArmState.armRadius,
                    commandedArmState.armAngle,
                    commandedWristState.wristPosition,
                    commandedWristTool,
                    commandedArmState.armVelocity,
                    ff
                )
    }
}