package org.team401.robot2019.config

import org.snakeskin.units.MagEncoderTicksPer100Ms
import org.snakeskin.units.RadiansPerSecond

object ControlParameters{
    object ArmParameters{
        val HOMING_VELOCITY = 100.MagEncoderTicksPer100Ms
        const val MAX_ACCELERATION = 0
        const val MAX_VELOCITY = 4098/8
        const val HOMED_POSITION = 0
    }
}