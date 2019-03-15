package org.team401.robot2019.config

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object HardwareMap {
    object Drivetrain {
        const val shifterSolenoid = 0

        const val leftRearSparkMaxId = 1
        const val leftMidSparkMaxId = 2
        const val leftFrontSparkMaxId = 3
        const val rightRearSparkMaxId = 4
        const val rightMidSparkMaxId = 5
        const val rightFrontSparkMaxId = 6


        const val pigeonImuId = 1
    }

    object Arm {
        const val brakeSolenoidId = 4

        const val leftIntakeWheelTalonId = 30
        const val rightIntakeWheelTalonId = 31

        const val pivotLeftTalonId = 20
        const val pivotRightTalonId = 21

        const val wristTalonId = 24
        const val extensionTalonId = 25
    }

    object Wrist{
        const val clawSolenoidID = 2
        const val cargoClawSolenoidID = 1

        const val ballSensorNCPort = 1
        const val ballSensorNOPort = 0

        const val potPort = 1 //analog

    }

    object FloorPickup {
        const val solenoidId = 3
        const val intakeWheelsTalonId = 31
    }

    object Climber {
        const val frontTalonId = 60
        const val backTalonId = 61
    }
}