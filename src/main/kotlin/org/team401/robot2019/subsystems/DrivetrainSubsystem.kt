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
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.RadiansPerSecondPerSecond
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.DriverstationDisplay
import org.team401.robot2019.LeftStick
import org.team401.robot2019.RightStick
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.config.Physics
import org.team401.taxis.diffdrive.component.IPathFollowingDiffDrive
import org.team401.taxis.diffdrive.component.impl.PigeonPathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.NonlinearFeedbackPathController
import org.team401.taxis.diffdrive.odometry.OdometryTracker
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */

object DrivetrainSubsystem: Subsystem(500L), IPathFollowingDiffDrive<SparkMaxCTRESensoredGearbox> by PigeonPathFollowingDiffDrive(
    SparkMaxCTRESensoredGearbox(
        WristSubsystem.leftIntakeTalon,
        CANSparkMax(HardwareMap.Drivetrain.leftFrontSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.Drivetrain.leftMidSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.Drivetrain.leftRearSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless)
    ),
    SparkMaxCTRESensoredGearbox(
        WristSubsystem.rightIntakeTalon,
        CANSparkMax(HardwareMap.Drivetrain.rightFrontSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.Drivetrain.rightMidSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.Drivetrain.rightRearSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless)
    ),
    PigeonIMU(HardwareMap.Drivetrain.pigeonImuId),
    Geometry.DrivetrainGeometry,
    Physics.DrivetrainDynamics,
    NonlinearFeedbackPathController(2.0, 0.7)
) {
    object ShifterStates: ShifterState(false, false)

    private val shifter = Solenoid(HardwareMap.Drivetrain.shifterSolenoid)

    /**
     * Shifts the drivetrain to the selected gear.  The available shifter states are available in ShifterStates.
     */
    fun shift(state: Boolean) {
        shifter.set(state)
    }

    enum class DriveStates {
        DisabledForFault,
        OpenLoopOperatorControl,
        PathFollowing,
        ClimbPull,
        ClimbStop,
        ClimbReposition,
    }

    enum class DriveFaults {
        MotorControllerReset
    }

    private val cheesyController = CheesyDriveController(ControlParameters.DrivetrainCheesyDriveParameters)

    val stateEstimator = OdometryTracker(this)

    const val gearRatioHigh = (50.0/28.0) * (64.0 / 15.0)

    val driveMachine: StateMachine<DriveStates> = stateMachine {
        state(DriveStates.DisabledForFault) {
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
                val kP = SmartDashboard.getNumber("driveP", 0.0)
                val kI = SmartDashboard.getNumber("driveI", 0.0)
                val kD = SmartDashboard.getNumber("driveD", 0.0)
                val kF = 0.0
                
                both {
                    master.pidController.p = kP
                    master.pidController.i = kI
                    master.pidController.d = kD
                    master.pidController.ff = kF
                }
                
                DrivetrainSubsystem.setPose(Pose2d(5.5 * 12.0, 17.25 * 12.0, Rotation2d.fromDegrees(0.0)))
                both {
                    setDeadband(0.0)
                }
                pathManager.reset()
                pathManager.setTrajectory(
                    TrajectoryIterator(TimedView(
                        pathManager.generateTrajectory(
                            false,
                            listOf(
                                Pose2d(5.5 * 12.0, 17.25 * 12.0, Rotation2d.fromDegrees(0.0)),
                                Pose2d(17.5 * 12.0, 20.0 * 12.0, Rotation2d.fromDegrees(0.0)),
                                Pose2d(20.0 * 12.0, 25.0 * 12.0, Rotation2d.fromDegrees(145.0))
                            ),
                            listOf(CentripetalAccelerationConstraint(110.0)),
                            14.0 * 12.0,
                            8.0 * 12.0,
                            9.0
                        )
                    ))
                )
            }

            rtAction {
                val output = pathManager.update(time, driveState.getFieldToVehicle(time))

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

                SmartDashboard.putNumber("leftDesired", leftVelocityRpm)
                SmartDashboard.putNumber("rightDesired", rightVelocityRpm)
                SmartDashboard.putNumber("leftActual", left.master.encoder.velocity)
                SmartDashboard.putNumber("rightActual", right.master.encoder.velocity)
                SmartDashboard.putNumber("leftFf", totalFfLeft)
                SmartDashboard.putNumber("rightFf", totalFfRight)
            }

            exit {
                stop()
            }
        }

        state(DriveStates.ClimbPull){
            entry {
                shift(ControlParameters.DrivetrainParameters.climbPullGear)
                arcade(ControlParameters.DrivetrainParameters.climbPullPower, 0.0)
            }
        }

        state(DriveStates.ClimbStop){
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
                DriverstationDisplay.climbRepositionModeEnabled.setBoolean(true)
            }
            action {
                arcade(LeftStick.readAxis { PITCH } * ControlParameters.DrivetrainParameters.slowingFactor,
                    RightStick.readAxis { ROLL } * ControlParameters.DrivetrainParameters.slowingFactor)
            }
            exit{
                DriverstationDisplay.climbRepositionModeEnabled.setBoolean(false)
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
        //Detect faults
        if (left.master.getStickyFault(CANSparkMax.FaultID.kHasReset) || left.slaves.any { it.getStickyFault(CANSparkMax.FaultID.kHasReset) } ||
            right.master.getStickyFault(CANSparkMax.FaultID.kHasReset) || right.slaves.any {
                it.getStickyFault(
                    CANSparkMax.FaultID.kHasReset
                )
            }) {
            fault(DriveFaults.MotorControllerReset)
            DriverStation.reportWarning("[Fault] A drive motor controller has reset!", false)
        }

        //Respond to faults
        if (isFaulted(DriveFaults.MotorControllerReset)) {
            driveMachine.setState(DriveStates.DisabledForFault).waitFor()
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

        //debug
        //println("Left: ${left.getPosition().toDegrees()}\t Right: ${right.getPosition().toDegrees()}")
        //println(driveState.getLatestFieldToVehicle().value)
    }

    override fun setup() {
        configureDriveMotorControllers()

        both {
            setPosition(0.0.Radians)
        }

        on(Events.TELEOP_ENABLED) {
            driveMachine.setState(DriveStates.OpenLoopOperatorControl)
        }

        SmartDashboard.putNumber("driveP", 0.0)
        SmartDashboard.putNumber("driveI", 0.0)
        SmartDashboard.putNumber("driveD", 0.0)
    }
}