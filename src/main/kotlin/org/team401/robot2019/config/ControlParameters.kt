package org.team401.robot2019.config

import org.snakeskin.units.*

object ControlParameters{
    object ArmParameters{
        //val HOMING_VELOCITY = 0.25.RevolutionsPerSecond
        const val HOMING_CURRENT = 0.8
        const val MAX_ACCELERATION = 4098.0
        const val MAX_VELOCITY = 4098.0/8
        const val HOMED_POSITION = 0


        val MIN_POS = 0.57.Radians.value
        val MAX_POS = 3.5.Radians.value

        val MIN_ARM_LENGTH = 38.5.Inches

    }
}