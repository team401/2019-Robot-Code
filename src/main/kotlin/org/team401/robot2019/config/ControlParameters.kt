package org.team401.robot2019.config

import org.snakeskin.logic.scalars.Scalar
import org.snakeskin.logic.scalars.ScalarGroup
import org.snakeskin.logic.scalars.SquareScalar
import org.snakeskin.measure.*
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.snakeskin.template.PIDFTemplate
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.control.superstructure.geometry.Point2d
import kotlin.math.roundToInt

object ControlParameters{
    object ArmParameters{
        /**
         * Max velocity and acceleration values to configure the extension motion magic controller
         */
        val extensionVelocity = 36.0.InchesPerSecond


        val extensionAcceleration = (36.0 * 3.5).InchesPerSecond //PER SECOND


        val ROTATION_MAX_ACCELERATION = 0.25.RevolutionsPerSecondPerSecond.toRadiansPerSecondPerSecond()
        val ROTATION_MAX_VELOCITY = 0.25.RevolutionsPerSecond.toRadiansPerSecond()


        val MIN_POS = 0.57.Radians.value
        val MAX_POS = 3.5.Radians.value

        /**
         * Voltage required to hold the superstructure static, divided by the cosine of the angle times the radius the test was taken at
         */
        const val kS = 0.0412

        /**
         * Velocity feedforward voltage.  "Voltage to velocity relationship"
         */
        const val kV = .05

        /**
         * Amount of time to home the arm
         */
        val extensionHomingTime = 0.25.Seconds

        /**
         * Power to home the arm at.  This should be negative
         */
        val extensionHomingPower = -0.1

        object ArmRotationPIDF: PIDFTemplate{
            override val kP = 0.0
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0 //THIS SHOULD ALWAYS BE ZERO!
        }

        object ArmExtensionPIDF: PIDFTemplate{
            override val kP = 0.5
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.24
        }
    }

    object WristParameters{
        val acceleration = 2.0.RevolutionsPerSecondPerSecond
        val cruiseVelocity = 2.0.RevolutionsPerSecond

        object WristRotationPIDF: PIDFTemplate {
            override val kP = 2.3
            override val kI = 0.0
            override val kD = 20.0
            override val kF = 0.84
        }
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
        val homingPower = -0.15

        /**
         * Number of seconds the legs must be at zero velocity until the system is considered homed
         */
        val homingTime = 0.25.Seconds
    }

    object ClimberPositions {
        val stowed = (-.5).Inches
        val l2Climb = (10.0).Inches
        val l3Climb = (20.0).Inches
    }
}