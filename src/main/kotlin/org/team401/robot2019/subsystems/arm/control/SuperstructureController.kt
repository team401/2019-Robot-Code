package org.team401.robot2019.subsystems.arm.control

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.subsystems.arm.geometry.WristState
import org.team401.robot2019.subsystems.arm.geometry.ArmState

object SuperstructureController {
    var output = SuperstructureControlOutput(0.0.Radians, 0.0.Inches, 0.0.Radians, 0.0)
    private set

    private fun calculateCounterbalanceVoltage(commandedArmState: ArmState): Double {
        val radius = commandedArmState.armRadius.value
        val theta = commandedArmState.armAngle.value

        //Equation: kS * r * theta
        //Constant kS: Voltage required to hold the arm static, divided by r * theta that the test was performed at
        return ControlParameters.ArmParameters.kS * radius * Math.cos(theta)
    }

    private fun calculateVelocityVoltage(commandedArmState: ArmState): Double {
        val velocity = commandedArmState.armVelocity.value

        //Equation: kV * desiredVelocity
        //Constant kV: Voltage to velocity relationship (measured after kS
        return ControlParameters.ArmParameters.kV * velocity
    }

    /**
     * @param commandedArmState The desired state of the arm
     * @param commandedWristState The desired state of the wrist
     *
     * Updates the field "output" with the current control command
     */
    fun update(commandedArmState: ArmState, commandedWristState: WristState) {
        //Calculate dynamics "counterbalance voltage"
        val counterbalanceFf = calculateCounterbalanceVoltage(commandedArmState)

        //Calculate velocity feedforward
        val velocityFf = calculateVelocityVoltage(commandedArmState)

        //Calculate total feedforward
        val ff = counterbalanceFf + velocityFf

        //Update control output
        output = SuperstructureControlOutput(
            commandedArmState.armAngle,
            commandedArmState.armRadius,
            commandedWristState.wristPosition,
            ff
        )
    }
}