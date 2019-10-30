package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.sensors.PigeonIMU
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ControlType
import edu.wpi.first.wpilibj.*
import org.snakeskin.component.impl.SparkMaxCTRESensoredGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.hardware.Hardware
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.RadiansPerSecondPerSecond
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.*
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.config.Physics
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.SuperstructureSideManager
import org.team401.robot2019.control.superstructure.geometry.VisionHeightMode
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.control.vision.*
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.util.LEDManager
import org.team401.robot2019.vision2.LimelightCameraEnhanced
import org.team401.robot2019.vision2.VisionSolver
import org.team401.taxis.diffdrive.component.IPathFollowingDiffDrive
import org.team401.taxis.diffdrive.component.impl.PigeonPathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.NonlinearFeedbackPathController
import org.team401.taxis.diffdrive.odometry.OdometryTracker
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import kotlin.math.ln
import kotlin.math.pow
import kotlin.math.tan

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */

object DrivetrainSubsystem: Subsystem(100L), IPathFollowingDiffDrive<SparkMaxCTRESensoredGearbox> by PigeonPathFollowingDiffDrive(
    SparkMaxCTRESensoredGearbox(
        Encoder(0, 1, false, CounterBase.EncodingType.k4X),
        CANSparkMax(HardwareMap.CAN.drivetrainLeftFrontSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.CAN.drivetrainLeftMidSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless),
        CANSparkMax(HardwareMap.CAN.drivetrainLeftRearSparkMaxId, CANSparkMaxLowLevel.MotorType.kBrushless)
    ),
    SparkMaxCTRESensoredGearbox(
        Encoder(2, 3, false, CounterBase.EncodingType.k4X),
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

    private val visionTuningButton = DigitalInput(9)

    private fun updateVisionDebug() {
        val activeCamera = VisionManager.frontCamera

        if (!visionTuningButton.get()) {
            activeCamera.configForVision(1)
            activeCamera.setLedMode(LimelightCamera.LedMode.UsePipeline)
            val area = activeCamera.getArea()

            println("Area: $area")
        } else {
            activeCamera.setLedMode(LimelightCamera.LedMode.Off)
        }
    }

    /**
     * Shifts the drivetrain to the selected gear.  The available shifter states are available in ShifterStates.
     */
    fun shift(state: Boolean) {
        shifter.set(state)
    }

    private fun getScoreReversePercent(): Double {
        return if (Gamepad.readButton { RIGHT_BUMPER } && SuperstructureController.output.wristTool == WristMotionPlanner.Tool.HatchPanelTool) {
            when (SuperstructureRoutines.side) {
                SuperstructureRoutines.Side.FRONT -> -0.25
                SuperstructureRoutines.Side.BACK -> 0.25
            }
        } else {
            0.0
        }
    }

    enum class DriveStates(val requiresSensors: Boolean = false) {
        EStopped,
        Disabled,
        ExternalControl,
        OpenLoopOperatorControl,
        PathFollowing(true),
        ClimbReposition,
        VisionAlign
    }

    enum class DriveFaults {
        MotorControllerReset,
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
                val pitch = LeftStick.readAxis { PITCH }
                val roll = RightStick.readAxis { ROLL }

                val output = cheesyController.update(
                    pitch,
                    roll,
                    false,
                    RightStick.readButton { TRIGGER }
                )

                val scoreAdjust = getScoreReversePercent()

                tank(output.left + scoreAdjust, output.right + scoreAdjust)

                val poseX = driveState.getLatestXInches()

                SuperstructureRoutines.sideManager.reportDriveCommand(Hardware.getRelativeTime(), pitch, poseX)
                SuperstructureRoutines.onSideManagerUpdate()
            }
        }

        val pathFollowingKp = 0.0
        val pathFollowingKd = 0.0

        state (DriveStates.PathFollowing) {
            entry {
                both {
                    setDeadband(0.0)
                }
            }

            rtAction {
                val output = pathManager.update(time, driveState.getFieldToVehicle(time))

                val leftVelocityActual = left.getVelocity().value
                val rightVelocityActual = right.getVelocity().value
                val errorLeft = output.left_velocity - leftVelocityActual
                val errorRight = output.right_velocity - rightVelocityActual

                val leftVelCorrection = errorLeft * pathFollowingKp * 12.0
                val rightVelCorrection = errorRight * pathFollowingKp * 12.0

                val leftAccelCorrection = output.left_accel * pathFollowingKd * 12.0
                val rightAccelCorrection = output.right_accel * pathFollowingKd * 12.0

                val leftOut = output.left_feedforward_voltage + leftVelCorrection + leftAccelCorrection
                val rightOut = output.right_feedforward_voltage + rightVelCorrection + rightAccelCorrection

                //Use velocity PID mode with gains all set to zero, simply to force the closed loop ramp rate (0.0)
                //and voltage compensation using arbFF.  No actual velocity control is being done on the SPARK.
                left.master.pidController.setReference(0.0, ControlType.kVelocity, 0, leftOut)
                right.master.pidController.setReference(0.0, ControlType.kVelocity, 0, rightOut)
            }
        }

        state(DriveStates.ClimbReposition) {
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

        state(DriveStates.VisionAlign) {
            lateinit var activeSide: SuperstructureRoutines.Side
            lateinit var activeCamera: LimelightCameraEnhanced
            lateinit var activeHeightMode: VisionHeightMode

            entry {
                SuperstructureRoutines.sideManager.reportAction(SuperstructureSideManager.Action.VISION_STARTED)
                SuperstructureRoutines.onSideManagerUpdate()
                activeSide = SuperstructureRoutines.side
                activeHeightMode = SuperstructureController.output.visionHeightMode
                activeCamera = when (activeSide) {
                    SuperstructureRoutines.Side.FRONT -> {
                        VisionManager.frontCamera
                    }
                    SuperstructureRoutines.Side.BACK -> {
                        VisionManager.backCamera
                    }
                }

                activeCamera.configForVision(activeHeightMode.pipeline)
                activeCamera.setLedMode(LimelightCamera.LedMode.UsePipeline)
                activeCamera.resetFrame()
                cheesyController.reset()

                VisionSolver.selectRegression(activeSide, activeHeightMode)
            }

            rtAction {
                val seesTarget = activeCamera.seesTarget()
                val targetAngle = activeCamera.entries.tx.getDouble(0.0)
                val targetArea = activeCamera.getArea()
                val output = cheesyController.update(
                    LeftStick.readAxis { PITCH },
                    RightStick.readAxis { ROLL },
                    false,
                    RightStick.readButton { TRIGGER }
                )
                val scoreAdjust = getScoreReversePercent()

                val correctionAngle = VisionSolver.solve(seesTarget, targetArea, targetAngle, activeCamera.robotToCamera)
                if (!correctionAngle.isNaN()) {
                    //Vision has found a valid target
                    when (activeSide) {
                        SuperstructureRoutines.Side.BACK -> LEDManager.setTrussLedMode(LEDManager.TrussLedMode.BlueSideActiveLockVision)
                        SuperstructureRoutines.Side.FRONT -> LEDManager.setTrussLedMode(LEDManager.TrussLedMode.RedSideActiveLockVision)
                    }
                    val adjustment = (correctionAngle * ControlParameters.DrivetrainParameters.visionKp) / 100.0
                    tank(output.left - adjustment + scoreAdjust, output.right + adjustment + scoreAdjust)
                } else {
                    //Vision has not found a valid target
                    when (activeSide) {
                        SuperstructureRoutines.Side.BACK -> LEDManager.setTrussLedMode(LEDManager.TrussLedMode.BlueSideActiveLock)
                        SuperstructureRoutines.Side.FRONT -> LEDManager.setTrussLedMode(LEDManager.TrussLedMode.RedSideActiveLock)
                    }
                    tank(output.left + scoreAdjust, output.right + scoreAdjust)
                }
            }

            exit {
                activeCamera.setLedMode(LimelightCamera.LedMode.Off)
                SuperstructureRoutines.sideManager.reportAction(SuperstructureSideManager.Action.VISION_FINISHED)
                SuperstructureRoutines.onSideManagerUpdate()
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
            master.pidController.p = 0.0
            master.pidController.i = 0.0
            master.pidController.d = 0.0
            master.pidController.ff = 0.0
            master.motorType = CANSparkMaxLowLevel.MotorType.kBrushless
            slaves.forEach {
                it.idleMode = CANSparkMax.IdleMode.kBrake
            }

            left.inverted = true
            right.inverted = false
        }
    }

    override fun action() {
        //updateVisionDebug()

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
        if (imu.state != PigeonIMU.PigeonState.Ready) {
            fault(DriveFaults.IMUFailure)
            DriverStation.reportWarning("[Fault] IMU failed!", false)
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

        if (isFaulted(DriveFaults.IMUFailure)) {
            if (imu.state == PigeonIMU.PigeonState.Ready) {
                clearFault(DriveFaults.IMUFailure)
                println("[Fault Cleared] IMU restored")
            }
        }

        //Insert debug println statements below:
        //println("left: ${left.getPosition()}  right: ${right.getPosition()}")
    }

    override fun setup() {
        configureDriveMotorControllers()

        both {
            setPosition(0.0.Radians)
        }

        setPose(Pose2d.identity())

        on (RobotEvents.HatchAcquired) {
            if (DriverStation.getInstance().isOperatorControl) {
                val ssPosition = ArmKinematics.forward(ArmSubsystem.getCurrentArmState())

                //If we're in teleop, reset drive pose on successful hatch intake.
                //This is used by the superstructure side manager to estimate the back hatch cycle
                if (ssPosition.x >= 0.0.Inches) {
                    //Front side is active, facing backwards
                    //13 inches from wheels to wrist pivot + x coord from wrist pivot to robot center
                    setPose(Pose2d(13.0 + ssPosition.x.value, 0.0, Rotation2d.fromDegrees(180.0)))
                } else {
                    //Back side is active, facing forwards
                    //13 inches from wheels to wrist pivot - x coord from wrist pivot to robot center
                    setPose(Pose2d(13.0 - ssPosition.x.value, 0.0, Rotation2d.identity()))
                }
            }
        }

        on (Events.TELEOP_ENABLED) {
            driveMachine.setState(DriveStates.OpenLoopOperatorControl)
            //On teleop enabled, set the pose to a very high magnitude negative number
            //This ensures that the superstructure side manager won't think we're doing a back
            //hatch cycle by mistake if the pose is somehow close to that threshold coming into teleop.
            setPose(Pose2d(-1e6, 0.0, Rotation2d.identity()))
        }
    }
}
