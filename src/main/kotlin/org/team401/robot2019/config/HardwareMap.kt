package org.team401.robot2019.config

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object HardwareMap {
    object Drivetrain {
        const val leftFrontTalonId = 0
        const val leftMidFTalonId = 1
        const val leftMidRTalonId = 2
        const val leftRearTalonId = 3
        const val rightFrontTalonId = 4
        const val rightMidFTalonId = 5
        const val rightMidRTalonId = 6
        const val rightRearTalonId = 7

        const val pigeonImuId = leftMidFTalonId
    }
}