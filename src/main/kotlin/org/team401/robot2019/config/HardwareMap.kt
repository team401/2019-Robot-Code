package org.team401.robot2019.config

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object
HardwareMap {
    /**
     * Holds IDs for all pneumatic components connected to the PCM
     */
    object Pneumatics {
        const val drivetrainShifterSolenoidId = 0
        const val hatchClawSolenoidId = 2
        const val cargoClawSolenoidId = 1
        const val floorPickupSolenoid = 3
    }

    /**
     * Holds IDs for all devices on CAN
     */
    object CAN {
        const val drivetrainLeftRearSparkMaxId = 1
        const val drivetrainLeftMidSparkMaxId = 2
        const val drivetrainLeftFrontSparkMaxId = 3
        const val drivetrainRightRearSparkMaxId = 4
        const val drivetrainRightMidSparkMaxId = 5
        const val drivetrainRightFrontSparkMaxId = 6
        const val drivetrainPigeonImuId = 1

        const val wristLeftIntakeWheelTalonId = 30
        const val wristRightIntakeWheelTalonId = 31
        const val wristPivotTalonId = 24

        const val armExtensionTalonId = 25
        const val armPivotLeftTalonId = 20
        const val armPivotRightTalonId = 21

        const val floorPickupWheelsVictorId = 1

        const val climbingFrontTalonId = 60
        const val climbingBackTalonId = 61
    }

    /**
     * Holds IDs for all DIO sensors and components
     */
    object DIO {

    }

    /**
     * Holds IDs for all analog sensors
     */
    object Analog {

    }

    /**
     * Holds PDP channel ID's
     */
    object PDP {
        const val floorPickupWheelsVictorChannel = 10
    }
}