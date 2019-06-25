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
        val armEncoderValueAtVertical by Selectable(1800, 2278)


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
        val velocity = 2.0.RevolutionsPerSecond
        val acceleration = 2.0.RevolutionsPerSecondPerSecond

        /**
         * Intake/scoring powers.  Negative values will spin the wheels clockwise (intake)
         */
        const val intakePower = -1.0
        const val holdingPower = -0.1
        const val scoringPower = 1.0

        /**
         * PIDF for the wrist rotation
         */
        object WristRotationPIDF: PIDFTemplate {
            override val kP by Selectable(8.0, 1.7) //comp, practice
            override val kI = 0.0
            override val kD by Selectable(100.0, 200.0) //comp wrist has friction brake, practice does not
            override val kF = 0.84
        }
    }

    /**
     * Setpoints for the superstructure
     */
    object SuperstructurePositions {
        //Distance to back setpoints up or down by to adjust them for the wrist being a different height on each side.
        val backHeightOffset = (1.5).Inches

        //Floor pickup positions
        val cargoFloorPickupFront = SuperstructureSetpoint.intakingCargo(
            Point2d(26.5.Inches, 7.0.Inches),
            0.0.Radians,
            VisionHeightMode.NONE
        ).withAngle((-10.0).Degrees.toRadians()).fromFloor()

        val cargoFloorPickupBack = cargoFloorPickupFront.flipped().upBy(3.0.Inches).withAngle(0.0.Radians)

        //Rocket cargo positions
        val rocketCargoBottomFront = SuperstructureSetpoint.holdingCargo(
            Point2d(31.0.Inches, 27.5.Inches),
            0.0.Radians,
            VisionHeightMode.LOW
        ).fromFloor()
        val rocketCargoMidFront = rocketCargoBottomFront.upBy(28.75.Inches).atX(12.0.Inches).withHeightMode(VisionHeightMode.MID)
        val rocketCargoHighFront = rocketCargoMidFront.upBy(17.0.Inches).atX((5.0).Inches).withAngle(45.0.Degrees.toRadians()).withHeightMode(VisionHeightMode.HIGH)

        val backCargoBottomOffset by Selectable(3.5.Inches, 1.5.Inches)
        val backCargoMidOffset by Selectable(2.0.Inches, 0.0.Inches)

        val rocketCargoBottomBack = rocketCargoBottomFront.flipped().upBy(backCargoBottomOffset)
        val rocketCargoMidBack = rocketCargoMidFront.flipped().upBy(backCargoMidOffset)
        val rocketCargoHighBack = rocketCargoHighFront.flipped()

        val hatchOffsetFront by Selectable((-1.0).Inches, 1.0.Inches)
        val hatchOffsetBack by Selectable(1.5.Inches, 0.5.Inches)

        //Intake hatch positions
        val hatchIntakeFront = SuperstructureSetpoint.intakingHatch(
            Point2d(28.5.Inches, 18.0.Inches),
            0.0.Radians,
            VisionHeightMode.LOW
        ).fromFloor().upBy(hatchOffsetFront)

        val hatchIntakeBack = hatchIntakeFront.flipped().upBy(hatchOffsetBack)

        //Rocket hatch positions
        val rocketHatchBottomFront = SuperstructureSetpoint.holdingHatch(
            Point2d(28.5.Inches, 19.0.Inches),
            0.0.Radians,
            VisionHeightMode.LOW
        ).fromFloor()
        val rocketHatchMidFront = rocketHatchBottomFront.upBy(28.0.Inches).atX(20.0.Inches).withHeightMode(VisionHeightMode.MID)
        val rocketHatchHighFront = rocketHatchMidFront.upBy(24.0.Inches).atX((16.5).Inches).withAngle(0.0.Degrees.toRadians()).withHeightMode(VisionHeightMode.HIGH)

        val rocketHatchBottomBack = rocketHatchBottomFront.flipped().upBy(3.0.Inches)
        val rocketHatchMidBack = rocketHatchMidFront.flipped()
        val rocketHatchHighBack = rocketHatchHighFront.flipped()

        val cargoShipCargoFront = SuperstructureSetpoint.holdingCargo(Point2d(16.0.Inches, 49.0.Inches), (-45.0).Degrees.toRadians(), VisionHeightMode.NONE).fromFloor()
        val cargoShipCargoBack = cargoShipCargoFront.flipped()

        val cargoIntakeLoadingStationFront = SuperstructureSetpoint.intakingCargo(
            Point2d(19.0.Inches, 23.5.Inches),
            0.0.Radians,
            VisionHeightMode.NONE
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

        val visionKp = 0.9
    }

    object FloorPickupParameters {
        const val intakeSpeed = 1.0
        const val ejectSpeed = -1.0

        val floorPickupAngle = 50.0.Degrees.toRadians()

        val floorPickupPoint = ArmKinematics.inverse(Point2d(18.0.Inches, (-17.0).Inches))

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
        val homingPower = -0.25

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
            return when (heightMode) {
                VisionHeightMode.NONE -> 0.0.Degrees

                VisionHeightMode.LOW -> {
                    when (side) {
                        SuperstructureRoutines.Side.FRONT -> {
                            when (tool) {
                                WristMotionPlanner.Tool.HatchPanelTool -> lowHatchAngularOffsetFront
                                WristMotionPlanner.Tool.CargoTool -> lowCargoAngularOffsetFront
                            }
                        }

                        SuperstructureRoutines.Side.BACK -> {
                            when (tool) {
                                WristMotionPlanner.Tool.HatchPanelTool -> lowHatchAngularOffsetBack
                                WristMotionPlanner.Tool.CargoTool -> lowCargoAngularOffsetBack
                            }
                        }
                    }
                }

                VisionHeightMode.MID -> {
                    when (side) {
                        SuperstructureRoutines.Side.FRONT -> {
                            when (tool) {
                                WristMotionPlanner.Tool.HatchPanelTool -> midHatchAngularOffsetFront
                                WristMotionPlanner.Tool.CargoTool -> midCargoAngularOffsetFront
                            }
                        }

                        SuperstructureRoutines.Side.BACK -> {
                            when (tool) {
                                WristMotionPlanner.Tool.HatchPanelTool -> midHatchAngularOffsetBack
                                WristMotionPlanner.Tool.CargoTool -> midCargoAngularOffsetBack
                            }
                        }
                    }
                }

                VisionHeightMode.HIGH -> {
                    when (side) {
                        SuperstructureRoutines.Side.FRONT -> {
                            when (tool) {
                                WristMotionPlanner.Tool.HatchPanelTool -> highHatchAngularOffsetFront
                                WristMotionPlanner.Tool.CargoTool -> highCargoAngularOffsetFront
                            }
                        }

                        SuperstructureRoutines.Side.BACK -> {
                            when (tool) {
                                WristMotionPlanner.Tool.HatchPanelTool -> highHatchAngularOffsetBack
                                WristMotionPlanner.Tool.CargoTool -> highCargoAngularOffsetBack
                            }
                        }
                    }
                }
            }
        }
    }
}