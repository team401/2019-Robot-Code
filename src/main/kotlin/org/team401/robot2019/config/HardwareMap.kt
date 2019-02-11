package org.team401.robot2019.config

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object HardwareMap {
    object Drivetrain {
        const val leftRearSparkMaxId = 1
        const val leftMidSparkMaxId = 2
        const val leftFrontSparkMaxId = 3
        const val rightRearSparkMaxId = 4
        const val rightMidSparkMaxId = 5
        const val rightFrontSparkMaxId = 6


        const val pigeonImuId = 1
    }

    object Arm {
        const val leftIntakeWheelId = 30
        const val rightIntakeWheelTalonId = 31
    }

    object FloorPickup {
        const val solenoidId = 1
        const val intakeWheelsTalonId = 31
    }
}