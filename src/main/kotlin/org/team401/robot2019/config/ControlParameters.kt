package org.team401.robot2019.config

import org.snakeskin.logic.scalars.Scalar
import org.snakeskin.logic.scalars.ScalarGroup
import org.snakeskin.logic.scalars.SquareScalar
import org.snakeskin.measure.*
import org.snakeskin.template.PIDFTemplate
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.control.superstructure.geometry.Point2d

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

        /**
         * Voltage required to hold the superstructure static, divided by the cosine of the angle times the radius the test was taken at
         */
        const val kS = 1.0

        /**
         * Velocity feedforward voltage.  "Voltage to velocity relationship"
         */
        const val kV = 1.0
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

    object FloorPickupParameters {
        const val intakeSpeed = 0.50
        const val ejectSpeed = -0.50
    }

    object ClimberParameters {
        object BackDownPIDF: PIDFTemplate {
            override val kP = 0.0
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0
        }

        object BackUpPIDF: PIDFTemplate {
            override val kP = 0.0
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0
        }

        object FrontDownPIDF: PIDFTemplate {
            override val kP = 0.0
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0
        }

        object FrontUpPIDF: PIDFTemplate {
            override val kP = 0.0
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0
        }

        val climberVelocity = 12.0.InchesPerSecond
        val climberAcceleration = 24.0.InchesPerSecond //per second.  We are using a velocity unit here because the unit library can't convert lin -> ang accel

        /**
         * Power to apply to the legs to home them.  This should be negative
         */
        val homingPower = -0.25

        /**
         * Number of seconds the legs must be at zero velocity until the system is considered homed
         */
        val homingTime = 0.5.Seconds
    }

    object ClimberPositions {
        /**
         * Distance from the ground that the legs are at when homed.  These are used to offset the home position
         * so that zero is right on the ground.  These constants should be negative
         */
        val frontHomeOffset = (-2.5).Inches
        val backHomeOffset = (-0.5).Inches

        val stowed = (-.5).Inches
        val l2Climb = (10.0).Inches
        val l3Climb = (20.0).Inches
    }
}