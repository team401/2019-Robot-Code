package org.team401.robot2019.config

import org.snakeskin.logic.scalars.Scalar
import org.snakeskin.logic.scalars.ScalarGroup
import org.snakeskin.logic.scalars.SquareScalar
import org.snakeskin.measure.*
import org.snakeskin.template.PIDFTemplate
import org.snakeskin.utility.CheesyDriveController
import org.snakeskin.utility.Selectable
import org.team401.robot2019.control.superstructure.geometry.SuperstructureSetpoint
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.DrivetrainSubsystem

object ControlParameters{
    object ArmParameters{
        /**
         * The mechanical goons put the arm encoder on the incorrect side of the practice robot.
         * As a result, we have to do this now.  Thanks Patrick.
         */

        val armEncoderPhase by Selectable(false, true)

        /**
         * Raw, native reading of the pulse width sensor on the arm pivot when the arm is at 90 degrees (vertical)
         * This is used to set the offset correctly between the different robots.
         */
        val armEncoderValueAtVertical by Selectable(1800, 2278)

        /**
         * Max velocity and acceleration values to configure the extension motion magic controller
         */
        val extensionVelocity = 36.0.InchesPerSecond


        val extensionAcceleration = (36.0 * 3.5).InchesPerSecond //PER SECOND


        val ROTATION_MAX_ACCELERATION = 0.6.RevolutionsPerSecondPerSecond.toRadiansPerSecondPerSecond()
        val ROTATION_MAX_VELOCITY = 0.2.RevolutionsPerSecond.toRadiansPerSecond()


        val MIN_POS = 0.57.Radians.value
        val MAX_POS = 3.5.Radians.value

        /**
         * Voltage required to hold the superstructure static, divided by the cosine of the angle times the radius the test was taken at
         */
        const val kS = 0.03

        /**
         * Velocity feedforward voltage.  "Voltage to velocity relationship"
         */
        const val kV = 2.65

        /**
         * Amount of time to home the arm
         */
        val extensionHomingTime = 0.25.Seconds

        /**
         * Power to home the arm at.  This should be negative
         */
        val extensionHomingPower = -0.1

        object ArmRotationMovePIDF: PIDFTemplate{
            override val kP = 3.0
            override val kI = 0.0
            override val kD = 20.0
            override val kF = 0.0 //THIS SHOULD ALWAYS BE ZERO!
        }

        object ArmRotationVerticalPIDF: PIDFTemplate {
            override val kP = 2.5
            override val kI = 0.0
            override val kD = 20.0
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

        val hasCargoTime = 0.25.Seconds

        val intakePower = -1.0
        val holdingPower = -0.1
        val scoringPower = 1.0

        const val invertPot = true
        const val degreesPerPotValue = (180.0 / 805.0)
        const val potOffset = 1965

        object WristRotationPIDF: PIDFTemplate {
            override val kP = 1.6
            override val kI = 0.0
            override val kD = 200.0
            override val kF = 0.84
        }
    }

    object ArmPositions {
        //Floor pickup positions
        val cargoFloorPickupFront = SuperstructureSetpoint.intakingCargo(
            Point2d(26.0.Inches, 8.5.Inches),
            0.0.Radians
        ).withAngle((0.0).Degrees.toRadians()).fromFloor()

        val cargoFloorPickupBack = cargoFloorPickupFront.flipped()//.upBy((-1.0).Inches)

        //Rocket cargo positions
        val rocketCargoBottomFront = SuperstructureSetpoint.holdingCargo(
            Point2d(31.0.Inches, 27.5.Inches),
            0.0.Radians
        ).fromFloor()
        val rocketCargoMidFront = rocketCargoBottomFront.upBy(28.75.Inches).atX(12.0.Inches)
        val rocketCargoHighFront = rocketCargoMidFront.upBy(17.0.Inches).atX((5.0).Inches).withAngle(45.0.Degrees.toRadians())

        val rocketCargoBottomBack = rocketCargoBottomFront.flipped()
        val rocketCargoMidBack = rocketCargoMidFront.flipped()
        val rocketCargoHighBack = rocketCargoHighFront.flipped()

        //Intake hatch positions
        val hatchIntakeFront = SuperstructureSetpoint.intakingHatch(
            Point2d(32.0.Inches, 17.0.Inches),
            0.0.Radians
        ).fromFloor()

        val hatchIntakeBack = hatchIntakeFront.flipped().upBy(2.0.Inches)

        //Rocket hatch positions
        val rocketHatchBottomFront = SuperstructureSetpoint.holdingHatch(
            Point2d(34.0.Inches, 19.0.Inches),
            0.0.Radians
        ).fromFloor()
        val rocketHatchMidFront = rocketHatchBottomFront.upBy(28.0.Inches).atX(20.0.Inches)
        val rocketHatchHighFront = rocketHatchMidFront.upBy(24.0.Inches).atX((18.0).Inches).withAngle(0.0.Degrees.toRadians())

        val rocketHatchBottomBack = rocketHatchBottomFront.flipped()
        val rocketHatchMidBack = rocketHatchMidFront.flipped()
        val rocketHatchHighBack = rocketHatchHighFront.flipped()

        val cargoShipCargoFront = SuperstructureSetpoint.holdingCargo(Point2d(28.0.Inches, 49.0.Inches), (-45.0).Degrees.toRadians()).fromFloor()
        val cargoShipCargoBack = cargoShipCargoFront.flipped()
    }

    object DrivetrainParameters {
        object VelocityPIDFHigh: PIDFTemplate {
            override val kP = 0.0
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0 //THIS SHOULD ALWAYS BE ZERO!
        }

        /**
         * The power to drive onto the level 3 or 2 platforms
         */
        val climbPullPower = -0.5

        /**
         * Climb in low gear
         */
        val climbPullGear = DrivetrainSubsystem.ShifterStates.LOW

        val climbWheelStopDelay = 0.5.Seconds

        val slowingFactor = 1 / 2.0
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
            override val kP = .7
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0
        }

        object BackUpPIDF: PIDFTemplate {
            override val kP = .7
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0
        }

        object FrontDownPIDF: PIDFTemplate {
            override val kP = .9
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0
        }

        object FrontUpPIDF: PIDFTemplate {
            override val kP = .9
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0
        }

        /**
         * Velocities for the actual climb.  The climb will use the normal acceleration values.
         */
        val climberVelocityDownClimbBack = 10.0.InchesPerSecond
        val climberVelocityDownClimbFront = 14.0.InchesPerSecond

        val climberVelocityDown = 10.0.InchesPerSecond
        val climberAccelerationDown = 10.0.InchesPerSecond //per second.  We are using a velocity unit here because the unit library can't convert lin -> ang accel

        val climberVelocityUp = 36.0.InchesPerSecond
        val climberAccelerationUp = 72.0.InchesPerSecond //per second

        val climberVelocityUpSlow = climberVelocityDown
        val climberAccelerationUpSlow = climberAccelerationDown

        /**
         * Power to apply to the legs to home them.  This should be negative
         */
        val homingPower = -0.15

        /**
         * Number of seconds the legs must be at zero velocity until the system is considered homed
         */
        val homingTime = 0.25.Seconds

        /**
         * Tolerance for checking whether or not the climber is at the desired height
         */
        val climberTolerance = 2.0.Inches
    }

    object ClimberPositions {
        val stowed = (-.4).Inches
        val l2Climb = (10.0).Inches
        val l3Climb = (22.0).Inches
    }
}