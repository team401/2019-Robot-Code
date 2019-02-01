package org.team401.robot2019.subsystems.arm

import org.team401.robot2019.config.ControlParameters
import kotlin.math.cos


object ArmController{

    fun calculateRotationFF(commandedArmState: ArmState): Double{
        val radius = commandedArmState.position.first.value
        val theta = commandedArmState.position.second.value
        return ControlParameters.ArmParameters.kHold * radius * cos(theta)
    }
}