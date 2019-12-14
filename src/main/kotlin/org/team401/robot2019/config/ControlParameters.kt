package org.team401.robot2019.config

import org.snakeskin.logic.scalars.Scalar
import org.snakeskin.logic.scalars.ScalarGroup
import org.snakeskin.logic.scalars.SquareScalar
import org.snakeskin.measure.*
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.snakeskin.template.PIDFTemplate
import org.snakeskin.utility.CheesyDriveController
import org.snakeskin.utility.Selectable
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.SuperstructureSetpoint
import org.team401.robot2019.control.superstructure.geometry.VisionHeightMode
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.robot2019.subsystems.arm.control.ArmKinematics

object ControlParameters {
    /**
     * General control parameters for the extension and rotation of the arm
     */
    object ArmParameters {
        /**
         * The mechanical goons put the arm encoder on the incorrect side of the practice robot.
         * As a result, we have to do this now.  Thanks Patrick.
         */
        val armEncoderPhase by Selectable(false, true)

        /**
         * Raw, native reading of the pulse width sensor on the arm pivot when the arm is at 90 degrees (vertical)
         * This is used to set the offset correctly between the different robots.
         */
        // practice used to be 2278
        val armEncoderValueAtVertical by Selectable(2660, 2278)


        /**
         * Extension velocity and acceleration
         */
        val extensionVelocity = 36.0.InchesPerSecond
        val extensionAcceleration = (36.0 * 3.5).InchesPerSecond //PER SECOND

        /**
         * Rotation velocities and accelerations
         */
        val rotationAcceleration = 0.7.RevolutionsPerSecondPerSecond.toRadiansPerSecondPerSecond()
        val rotationVelocity = 0.6.RevolutionsPerSecond.toRadiansPerSecond()

        val rotationSlowAcceleration = 0.6.RevolutionsPerSecondPerSecond.toRadiansPerSecondPerSecond()
        val rotationSlowVelocity = 0.2.RevolutionsPerSecond.toRadiansPerSecond()

        /**
         * Voltage required to hold the superstructure static, divided by the cosine of the angle times the radius the test was taken at
         */
        const val kS = 0.03

        /**
         * Velocity feedforward voltage.  "Voltage to velocity relationship"
         */
        const val kV = 2.65

        /**
         * Amount of time that the extension must be still to consider it homed
         */
        val extensionHomingTime = 0.25.Seconds

        /**
         * Power to home the extension at.  This should be negative
         */
        val extensionHomingPower = -0.1

        /**
         * PID for a move of the arm to a standard setpoint
         */
        object ArmRotationMovePIDF: PIDFTemplate{
            override val kP = 3.0
            override val kI = 0.0
            override val kD = 20.0
            override val kF = 0.0 //THIS SHOULD ALWAYS BE ZERO!
        }

        /**
         * PID for a move of the arm to a vertical setpoint.
         * This should be less than the standard gain set to keep
         * the arm from "vibrating" in a vertical position (due to backlash)
         */
        object ArmRotationVerticalPIDF: PIDFTemplate {
            override val kP = 2.5
            override val kI = 0.0
            override val kD = 20.0
            override val kF = 0.0 //THIS SHOULD ALWAYS BE ZERO!
        }

        /**
         * PIDF for the arm extension controller.
         */
        object ArmExtensionPIDF: PIDFTemplate{
            override val kP = 0.5
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.24
        }
    }

    /**
     * General control parameters for the wrist, including the pivot and the wheels.
     */
    object WristParameters {
        /**
         * Wrist velocity and acceleration
         */
        val velocity = 0.75.RevolutionsPerSecond
        val acceleration = 2.0.RevolutionsPerSecondPerSecond

        /**
         * Intake/scoring powers.  Negative values will spin the wheels clockwise (intake)
         */
        const val cargoIntakePower = 1.0
        const val cargoHoldingPower = 0.1
        const val cargoScoringPower = -1.0

        const val hatchIntakePower = -1.0
        const val hatchHoldingPower = -0.1
        const val hatchScoringPower = 0.5

        /**
         * PIDF for the wrist rotation
         */
        object WristRotationPIDF: PIDFTemplate {
            override val kP by Selectable(8.0, 1.7) //comp, practice
            override val kI = 0.0
            override val kD by Selectable(800.0, 200.0) //comp wrist has friction brake, practice does not
            override val kF = 2.0
        }
    }

    /**
     * Setpoints for the superstructure
     */
    object SuperstructurePositions {
        //Floor pickup positions
        val cargoFloorPickupFront = SuperstructureSetpoint.intakingCargo(
            Point2d(23.5.Inches, 5.0.Inches),
            0.0.Radians,
            VisionHeightMode.NONE
        ).withAngle((5.0).Degrees.toRadians()).fromFloor()

        val cargoFloorPickupBack = cargoFloorPickupFront.flipped().upBy(1.0.Inches).withAngle((10.0).Degrees.toRadians())

        //Rocket cargo positions
        val rocketCargoBottomFront = SuperstructureSetpoint.holdingCargo(
            Point2d(24.0.Inches, 27.5.Inches),
            10.0.Degrees.toRadians(),
            VisionHeightMode.CARGO_SCORE_ROCKET
        ).fromFloor()
        val rocketCargoMidFront = rocketCargoBottomFront.upBy(27.75.Inches).atX(6.0.Inches)
        val rocketCargoHighFront = rocketCargoMidFront.upBy(19.0.Inches).atX((5.0).Inches).withAngle(45.0.Degrees.toRadians())

        val rocketCargoBottomBack = rocketCargoBottomFront.flipped()
        val rocketCargoMidBack = rocketCargoMidFront.flipped()
        val rocketCargoHighBack = rocketCargoHighFront.flipped()

        //Intake hatch positions
        val hatchIntakeFront = SuperstructureSetpoint.intakingHatch(
            Point2d(24.0.Inches, 20.0.Inches),
            (10.0).Degrees.toRadians(),
            VisionHeightMode.HATCH_INTAKE
        ).fromFloor()

        val hatchIntakeBack = hatchIntakeFront.flipped().withAngle(10.0.Degrees.toRadians())
            //.upBy((-1.0).Inches)

        //Rocket hatch positions
        val rocketHatchBottomFront = SuperstructureSetpoint.holdingHatch(
            Point2d(24.0.Inches, 21.0.Inches),
            (10.0.Degrees).toRadians(),
            VisionHeightMode.HATCH_SCORE
        ).fromFloor()
        val rocketHatchMidFront = rocketHatchBottomFront.upBy(28.0.Inches).atX(6.0.Inches).withAngle(10.0.Degrees.toRadians())
        val rocketHatchHighFront = rocketHatchMidFront.upBy(25.0.Inches).atX((6.0).Inches).withAngle(15.0.Degrees.toRadians())

        val rocketHatchBottomBack = rocketHatchBottomFront.flipped().withAngle(10.0.Degrees.toRadians())
        val rocketHatchMidBack = rocketHatchMidFront.flipped().withAngle(0.0.Radians)
        val rocketHatchHighBack = rocketHatchHighFront.flipped()

        val cargoShipCargoFront = SuperstructureSetpoint.holdingCargo(Point2d(16.0.Inches, 49.0.Inches), (-45.0).Degrees.toRadians(), VisionHeightMode.HATCH_SCORE).fromFloor()
        val cargoShipCargoBack = cargoShipCargoFront.flipped()

        val cargoIntakeLoadingStationFront = SuperstructureSetpoint.intakingCargo(
            Point2d(19.0.Inches, 23.5.Inches),
            0.0.Radians,
            VisionHeightMode.HATCH_INTAKE //hatch intake since we use the hatch loading station to get the ball
        )
        val cargoIntakeLoadingStationBack = cargoIntakeLoadingStationFront.flipped()
    }

    /**
     * General control parameters for the drivetrain
     */
    object DrivetrainParameters {
        /**
         * Velocity PID for the drive in high gear
         */
        object VelocityPIDFHigh: PIDFTemplate {
            override val kP = 0.0
            override val kI = 0.0
            override val kD = 0.0
            override val kF = 0.0 //THIS SHOULD ALWAYS BE ZERO!
        }

        /**
         * Control parameters for the cheesy drive controller in teleop
         */
        object CheesyDriveParameters: CheesyDriveController.DefaultParameters() {
            override val quickTurnScalar = ScalarGroup(SquareScalar, object : Scalar {
                override fun scale(input: Double) = input / 3.33
            })
        }

        /**
         * Limits on odometry error.  If these limits are exceeded it is safe to assume there is an issue
         * with one or more of the sensors
         */
        val maxOdometryTranslationError = 24.0.Inches
        val maxOdometryRotationError = 45.0.Degrees.toRadians()
        val maxOdometryErrorCycles = 10

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

        val visionKp = 0.8
    }

    object FloorPickupParameters {
        const val intakeSpeed = 1.0
        const val ejectSpeed = -1.0

        val floorPickupAngle = 16.171875.Degrees.toRadians()

        val floorPickupPoint = ArmKinematics.inverse(Point2d(21.40664.Inches, (-6.38621).Inches))

        const val floorPickupCurrentLimit = 4.0
        val floorPickupCurrentTimeout = 0.2.Seconds
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
        val homingPower = -0.35

        /**
         * Number of seconds the legs must be at zero velocity until the system is considered homed
         */
        val homingTime = 0.15.Seconds

        /**
         * Tolerance for checking whether or not the climber is at the desired height
         */
        val climberTolerance = 2.0.Inches
    }

    object ClimberPositions {
        val stowed = (-.4).Inches
        val l2Climb = (9.0).Inches
        val l3Climb = (22.0).Inches
    }

    /**
     * Contains the angular offests, in degrees, for vision tracking.
     */
    object VisionOffsets {
        val lowHatchAngularOffsetFront = 0.0.Degrees//12.16.Degrees
        val midHatchAngularOffsetFront = 0.0.Degrees//10.0.Degrees
        val highHatchAngularOffsetFront = 0.0.Degrees//14.18.Degrees

        val lowCargoAngularOffsetFront = 0.0.Degrees
        val midCargoAngularOffsetFront = 0.0.Degrees
        val highCargoAngularOffsetFront = 0.0.Degrees

        val lowHatchAngularOffsetBack = lowHatchAngularOffsetFront
        val midHatchAngularOffsetBack = midHatchAngularOffsetFront
        val highHatchAngularOffsetBack = highHatchAngularOffsetFront

        val lowCargoAngularOffsetBack = lowCargoAngularOffsetFront
        val midCargoAngularOffsetBack = midCargoAngularOffsetFront
        val highCargoAngularOffsetBack = highCargoAngularOffsetFront

        /**
         * Selects the appropriate offset given the side, height, and tool
         */
        fun select(side: SuperstructureRoutines.Side, heightMode: VisionHeightMode, tool: WristMotionPlanner.Tool): AngularDistanceMeasureDegrees {
            return 0.0.Degrees
        }
    }
}