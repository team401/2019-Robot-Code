package org.team401.robot2019.subsystems.arm

import org.team401.robot2019.config.ControlParameters
import kotlin.math.cos


object ArmController{

    fun calculateRotationFF(commandedArmState: ArmState): Double{
        val radius = commandedArmState.armRadius.value
        val theta = commandedArmState.armAngle.value
        return ControlParameters.ArmParameters.kHold * radius * cos(theta)
    }
}