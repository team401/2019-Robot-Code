package org.team401.robot2019.config

import org.snakeskin.logic.scalars.Scalar
import org.snakeskin.logic.scalars.ScalarGroup
import org.snakeskin.logic.scalars.SquareScalar
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.subsystems.arm.armsim.Point2d

object ControlParameters{
    object ArmParameters{
        //val HOMING_VELOCITY = 0.25.RevolutionsPerSecond
        const val HOMING_CURRENT = 0.8
        val MAX_ACCELERATION = 3.14.RadiansPerSecond
        val MAX_VELOCITY = 3.14.RadiansPerSecond
        const val HOMED_POSITION = 0

        //val DEFAULT_ARM_POSITION = Point2d(0.0.Inches, 4.0.Inches) // Use real numbers

        val MIN_POS = 0.57.Radians.value
        val MAX_POS = 3.5.Radians.value

        const val kHold = 0.0
    }
    object ArmPositions{
        val ROCKET_TOP = Point2d(0.0.Inches, 0.0.Inches)
        val ROCKET_MID = Point2d(0.0.Inches, 0.0.Inches)
        val ROCKET_LOW = Point2d(0.0.Inches, 0.0.Inches)
        val CARGO_SHIP = Point2d(0.0.Inches, 0.0.Inches)
        val LOADING_STATION = Point2d(0.0.Inches, 0.0.Inches)
        val HATCH_FLOOR_PICKUP = Point2d(0.0.Inches, 0.0.Inches)
    }

    object DrivetrainCheesyDriveParameters: CheesyDriveController.DefaultParameters() {
        override val quickTurnScalar = ScalarGroup(SquareScalar, object : Scalar {
            override fun scale(input: Double) = input / 3.33
        })
    }
}