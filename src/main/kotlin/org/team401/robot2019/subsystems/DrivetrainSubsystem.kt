package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.sensors.PigeonIMU
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ControlType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.component.impl.SparkMaxCTRESensoredGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.RadiansPerSecondPerSecond
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.DriverStationDisplay
import org.team401.robot2019.LeftStick
import org.team401.robot2019.RightStick
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.config.Physics
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.geometry.VisionHeightMode
import org.team401.robot2019.control.vision.*
import org.team401.robot2019.vision2.DifferentialKinematics
import org.team401.robot2019.vision2.LimelightCameraEnhanced
import org.team401.robot2019.vision2.RobotState
import org.team401.robot2019.vision2.RobotStateEstimator
import org.team401.taxis.diffdrive.component.IPathFollowingDiffDrive
import org.team401.taxis.diffdrive.component.impl.PigeonPathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.NonlinearFeedbackPathController
import org.team401.taxis.diffdrive.odometry.OdometryTracker
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import org.team401.taxis.geometry.Twist2d
import kotlin.math.abs
import kotlin.math.ln
import kotlin.math.min
import kotlin.math.tan

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */

object DrivetrainSubsystem: Subsystem(100L), IPathFollowingDiffDrive<SparkMaxCTRESensoredGearbox> by PigeonPathFollowingDiffDrive(
    SparkMaxCTRESensoredGearbox(
        CANSparkMax(HardwareMap.CAN.drivetrainLeftFrontSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.CAN.drivetrainLeftMidSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.CAN.drivetrainLeftRearSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless)
    ),
    SparkMaxCTRESensoredGearbox(
        CANSparkMax(HardwareMap.CAN.drivetrainRightFrontSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.CAN.drivetrainRightMidSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.CAN.drivetrainRightRearSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless)
    ),
    PigeonIMU(HardwareMap.CAN.drivetrainPigeonImuId),
    Geometry.DrivetrainGeometry,
    Physics.DrivetrainDynamics,
    NonlinearFeedbackPathController(2.0, 0.7)
) {
    object ShifterStates: ShifterState(false, false)

    private val shifter = Solenoid(HardwareMap.Pneumatics.drivetrainShifterSolenoidId)

    /**
     * Shifts the drivetrain to the selected gear.  The available shifter states are available in ShifterStates.
     */
    fun shift(state: Boolean) {
        shifter.set(state)
    }

    /**
     * Uses a calibrated logarithmic regression to calculate the distance to a target from the area percentage
     */
    fun calculateLowVisionTargetDistanceInches(area: Double): Double {
        return 72.8901074814 - 21.3227275179 * ln(area)
    }

    enum class DriveStates(val requiresSensors: Boolean = false) {
        EStopped,
        Disabled,
        ExternalControl,
        OpenLoopOperatorControl,
        PathFollowing(true),
        ClimbStop,
        ClimbReposition,
        VisionAlign,
        VisionTuning
    }

    enum class DriveFaults {
        MotorControllerReset,
        LeftEncoderFailure,
        RightEncoderFailure,
        IMUFailure
    }

    private val cheesyController = CheesyDriveController(ControlParameters.DrivetrainParameters.CheesyDriveParameters)

    val stateEstimator = OdometryTracker(this)

    const val gearRatioHigh = (50.0/28.0) * (64.0 / 15.0)

    val driveMachine: StateMachine<DriveStates> = stateMachine {
        rejectAllIf(*DriveStates.values()){isInState(DriveStates.EStopped)}

        state(DriveStates.ExternalControl) {}

        state(DriveStates.EStopped){
            entry {
                println("DRIVETRAIN E STOP")
                stop()
            }
            exit {
                println("Drivetrain Operational")
            }
        }
        state(DriveStates.Disabled) {
            entry {
                stop() //Send no more commands to the controllers
            }
        }

        state(DriveStates.OpenLoopOperatorControl) {
            entry {
                cheesyController.reset()
                shift(ShifterStates.HIGH)
            }

            action {
                val output = cheesyController.update(
                    LeftStick.readAxis { PITCH },
                    RightStick.readAxis { ROLL },
                    false,
                    RightStick.readButton { TRIGGER }
                )

                tank(output.left, output.right)
            }
        }

        state (DrivetrainSubsystem.DriveStates.PathFollowing) {
            entry {
                val kP = 0.0//SmartDashboard.getNumber("driveP", 0.0)
                val kI = 0.0//SmartDashboard.getNumber("driveI", 0.0)
                val kD = 0.0//SmartDashboard.getNumber("driveD", 0.0)
                val kF = 0.0
                
                both {
                    master.pidController.p = kP
                    master.pidController.i = kI
                    master.pidController.d = kD
                    master.pidController.ff = kF
                    setDeadband(0.0)
                }
            }

            rtAction {
                val output = pathManager.update(time, VisionState.getFieldToRobot(time))

                val leftVelocityRpm = output.left_velocity.RadiansPerSecond.toRevolutionsPerMinute().value * gearRatioHigh
                val rightVelocityRpm = output.right_velocity.RadiansPerSecond.toRevolutionsPerMinute().value * gearRatioHigh

                val leftAccelRpmPerMs = output.left_accel.RadiansPerSecondPerSecond.toRevolutionsPerMinutePerMillisecond().value * gearRatioHigh
                val rightAccelRpmPerMs = output.right_accel.RadiansPerSecondPerSecond.toRevolutionsPerMinutePerMillisecond().value * gearRatioHigh

                val leftFfVolts = output.left_feedforward_voltage
                val rightFfVolts = output.right_feedforward_voltage

                val totalFfLeft = leftFfVolts + (ControlParameters.DrivetrainParameters.VelocityPIDFHigh.kD * leftAccelRpmPerMs * 12.0)
                val totalFfRight = rightFfVolts + (ControlParameters.DrivetrainParameters.VelocityPIDFHigh.kD * rightAccelRpmPerMs * 12.0)

                left.master.pidController.setReference(leftVelocityRpm, ControlType.kVelocity, 0, totalFfLeft)
                right.master.pidController.setReference(rightVelocityRpm, ControlType.kVelocity, 0, totalFfRight)

                //println("vision: ${VisionState.getFieldToRobot(time)}  odo: ${driveState.getFieldToVehicle(time)}")
            }

            exit {
                //stop()
            }
        }

        state(DriveStates.ClimbStop) {
            action {
                if (ClimberSubsystem.backWithinTolerance(ControlParameters.ClimberPositions.stowed)){
                    Thread.sleep(ControlParameters.DrivetrainParameters.climbWheelStopDelay.toMilliseconds().value.toLong())
                    stop()
                    setState(DriveStates.ClimbReposition)
                }
            }
        }

        state(DriveStates.ClimbReposition){
            entry {
                DriverStationDisplay.climbRepositionModeEnabled.setBoolean(true)
            }
            action {
                arcade(LeftStick.readAxis { PITCH } * ControlParameters.DrivetrainParameters.slowingFactor,
                    RightStick.readAxis { ROLL } * ControlParameters.DrivetrainParameters.slowingFactor)
            }
            exit{
                DriverStationDisplay.climbRepositionModeEnabled.setBoolean(false)
            }
        }

        state(DrivetrainSubsystem.DriveStates.VisionAlign){
            lateinit var activeSide: SuperstructureRoutines.Side
            lateinit var activeCamera: LimelightCameraEnhanced

            entry {
                activeSide = SuperstructureRoutines.side
                activeCamera = when (activeSide) {
                    SuperstructureRoutines.Side.FRONT -> {
                        VisionManager.frontCamera
                    }
                    SuperstructureRoutines.Side.BACK -> {
                        VisionManager.backCamera
                    }
                }

                activeCamera.configForVision(1)
                activeCamera.setLedMode(LimelightCamera.LedMode.UsePipeline)
                activeCamera.resetFrame()
                cheesyController.reset()
            }

            val alignmentOffset = 30.0 //inches
            val degrees180 = Rotation2d.fromDegrees(180.0)
            val autosteerKp = .2

            rtAction {
                val seesTarget = activeCamera.seesTarget()
                val output = cheesyController.update(
                    LeftStick.readAxis { PITCH },
                    RightStick.readAxis { ROLL },
                    false,
                    RightStick.readButton { TRIGGER }
                )
                var outLeft = output.left
                var outRight = output.right
                if (seesTarget) {
                    val area = activeCamera.getArea()
                    val distance = calculateLowVisionTargetDistanceInches(area)
                    if (distance > 0.0) {
                        val targetAngle = -activeCamera.entries.tx.getDouble(0.0)
                        val cameraToTarget =
                            Pose2d.fromTranslation(Translation2d(distance, distance * tan(Math.toRadians(targetAngle))))
                        val robotToTarget = activeCamera.robotToCamera.transformBy(cameraToTarget)
                        var bearing = robotToTarget.translation.direction()
                        if (activeSide == SuperstructureRoutines.Side.BACK) {
                            bearing = bearing.rotateBy(degrees180)
                        }
                        if (bearing.degrees in -20.0..20.0) {
                            val adjustment = (bearing.degrees * ControlParameters.DrivetrainParameters.visionKp) / 100.0
                            outLeft -= adjustment
                            outRight += adjustment
                        }

                        println(bearing)
                    }
                }
                tank(outLeft, outRight)
            }

            exit {
                activeCamera.setLedMode(LimelightCamera.LedMode.Off)
            }
        }

        state (DriveStates.VisionTuning) {
            lateinit var activeSide: SuperstructureRoutines.Side
            lateinit var activeCamera: LimelightCamera

            entry {
                activeSide = SuperstructureRoutines.side
                activeCamera = when (activeSide) {
                    SuperstructureRoutines.Side.FRONT -> {
                        VisionManager.frontCamera
                    }
                    SuperstructureRoutines.Side.BACK -> {
                        VisionManager.backCamera
                    }
                }

                activeCamera.configForVision(3)
                activeCamera.setLedMode(LimelightCamera.LedMode.UsePipeline)
                activeCamera.resetFrame()
            }

            rtAction {
                val tx = activeCamera.entries.tx.getDouble(0.0)

                val output = cheesyController.update(
                    LeftStick.readAxis { PITCH },
                    RightStick.readAxis { ROLL },
                    false,
                    RightStick.readButton { TRIGGER }
                )

                tank(output.left, output.right)

                println("Camera tx: $tx")
            }

            exit {
                activeCamera.setLedMode(LimelightCamera.LedMode.Off)
            }
        }


        default {
            entry {
                stop()
            }
        }
    }

    private fun configureDriveMotorControllers() {
        both {
            master.idleMode = CANSparkMax.IdleMode.kBrake
            master.setSmartCurrentLimit(40)
            master.setControlFramePeriodMs(5)
            master.openLoopRampRate = 0.25
            master.closedLoopRampRate = 0.0
            master.pidController.p = ControlParameters.DrivetrainParameters.VelocityPIDFHigh.kP
            master.pidController.i = ControlParameters.DrivetrainParameters.VelocityPIDFHigh.kI
            master.pidController.d = ControlParameters.DrivetrainParameters.VelocityPIDFHigh.kD
            master.pidController.ff = ControlParameters.DrivetrainParameters.VelocityPIDFHigh.kF
            master.motorType = CANSparkMaxLowLevel.MotorType.kBrushless
            slaves.forEach {
                it.idleMode = CANSparkMax.IdleMode.kBrake
            }

            left.inverted = true
            right.inverted = false
        }
    }

    /**
     * Configures the "auxiliary" Talon SRX motor controllers that will be used to provide feedback information
     * to the drive.  This should be called by the subsystem that owns the Talons, Wrist at the time of writing.
     */
    fun configureFeedbackTalonsForDrive(leftTalon: TalonSRX, rightTalon: TalonSRX) {
        //Set feedback device
        leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100)
        rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100)

        //Set sensor phase
        leftTalon.setSensorPhase(true)
        rightTalon.setSensorPhase(true)

        //Configure status frame rate
        leftTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100)
        rightTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100)

        //Configure velocity measurement period and window
        leftTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 100)
        rightTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, 100)
        leftTalon.configVelocityMeasurementWindow(1, 100)
        rightTalon.configVelocityMeasurementWindow(1, 100)
    }

    override fun action() {
        // Detect faults

        //Motor controller reset fault
        if (left.master.getStickyFault(CANSparkMax.FaultID.kHasReset) || left.slaves.any { it.getStickyFault(CANSparkMax.FaultID.kHasReset) } ||
            right.master.getStickyFault(CANSparkMax.FaultID.kHasReset) || right.slaves.any {
                it.getStickyFault(
                    CANSparkMax.FaultID.kHasReset
                )
            }) {
            fault(DriveFaults.MotorControllerReset)
            DriverStation.reportWarning("[Fault] A drive motor controller has reset!", false)
        }

        //Sensor faults
        /*
        if (left.ctreController.sensorCollection.pulseWidthRiseToRiseUs == 0) {
            fault(DriveFaults.LeftEncoderFailure)
            DriverStation.reportWarning("[Fault] Left encoder failed!", false)
        }
        if (right.ctreController.sensorCollection.pulseWidthRiseToRiseUs == 0) {
            fault(DriveFaults.RightEncoderFailure)
            DriverStation.reportWarning("[Fault] Right encoder failed!", false)
        }
        */
        if (imu.state != PigeonIMU.PigeonState.Ready) {
            fault(DriveFaults.IMUFailure)
            DriverStation.reportWarning("[Fault] IMU failed!", false)
        }


        // Break state
        if (isFaulted(DriveFaults.LeftEncoderFailure, DriveFaults.RightEncoderFailure, DriveFaults.IMUFailure)) {
            //A sensor has failed, if we're in a state that requires sensors, drop into a safe state
            if ((driveMachine.getState() as? DriveStates)?.requiresSensors == true) {
                //The current state requires sensors, drop into manual control
                driveMachine.setState(DriveStates.OpenLoopOperatorControl)
            }
        }

        // Respond to faults

        //Motor controller reset fault
        if (isFaulted(DriveFaults.MotorControllerReset)) {
            driveMachine.setState(DriveStates.Disabled).waitFor()
            configureDriveMotorControllers() //Reconfigure motor controllers
            clearFault(DriveFaults.MotorControllerReset)
            both {
                master.clearFaults()
                slaves.forEach {
                    it.clearFaults()
                }
            }
            println("[Fault Cleared] Drive motor controllers reconfigured")
            driveMachine.setState(DriveStates.OpenLoopOperatorControl)
        }

        //Sensor faults
        /*
        if (isFaulted(DriveFaults.LeftEncoderFailure)) {
            if (left.ctreController.sensorCollection.pulseWidthRiseToRiseUs != 0) {
                clearFault(DriveFaults.LeftEncoderFailure)
                println("[Fault Cleared] Left encoder restored")
            }
        }
        if (isFaulted(DriveFaults.RightEncoderFailure)) {
            if (right.ctreController.sensorCollection.pulseWidthRiseToRiseUs != 0) {
                clearFault(DriveFaults.RightEncoderFailure)
                println("[Fault Cleared] Right encoder restored")
            }
        }
        */
        if (isFaulted(DriveFaults.IMUFailure)) {
            if (imu.state == PigeonIMU.PigeonState.Ready) {
                clearFault(DriveFaults.IMUFailure)
                println("[Fault Cleared] IMU restored")
            }
        }

        // Driver Station Shutoff
        /*
        if (DriverStationDisplay.driveStopped.getBoolean(false)) {
            driveMachine.setState(DriveStates.EStopped)
        }else if (driveMachine.isInState(DriveStates.EStopped) && !DriverStationDisplay.driveStopped.getBoolean(false)) {
            driveMachine.setState(DriveStates.Disabled)
        }
        */
        //Insert debug println statements below:
        //println("left: ${left.getPosition()}  right: ${right.getPosition()}")
    }

    override fun setup() {
        configureDriveMotorControllers()
        configureFeedbackTalonsForDrive(
            WristSubsystem.leftIntakeTalon,
            WristSubsystem.rightIntakeTalon
        )

        both {
            setPosition(0.0.Radians)
        }

        setPose(Pose2d.identity())

        on (Events.TELEOP_ENABLED) {
            driveMachine.setState(DriveStates.OpenLoopOperatorControl)
        }
    }
}
