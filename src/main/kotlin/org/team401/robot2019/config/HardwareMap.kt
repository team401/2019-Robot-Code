package org.team401.robot2019.config

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object HardwareMap {
    object Drivetrain {
        const val leftFrontTalonId = 1
        const val leftMidFTalonId = 2
        const val leftMidRTalonId = 3
        const val leftRearTalonId = 4
        const val rightFrontTalonId = 5
        const val rightMidFTalonId = 6
        const val rightMidRTalonId = 7
        const val rightRearTalonId = 8

        const val pigeonImuId = leftRearTalonId
    }
}