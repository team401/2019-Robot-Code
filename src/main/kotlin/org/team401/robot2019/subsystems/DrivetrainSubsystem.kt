package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlFrame
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import com.ctre.phoenix.sensors.PigeonIMU
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.ControlType
import edu.wpi.first.wpilibj.*
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.component.impl.SparkMaxCTRESensoredGearbox
import org.snakeskin.component.impl.TalonSRXVictorSensoredGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.hardware.Hardware
import org.snakeskin.logic.LockingDelegate
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
import org.team401.robot2019.util.fieldMirror
import org.team401.robot2019.vision2.LimelightCameraEnhanced
import org.team401.robot2019.vision2.VisionSolver
import org.team401.taxis.diffdrive.component.IPathFollowingDiffDrive
import org.team401.taxis.diffdrive.component.impl.PigeonPathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.NonlinearFeedbackPathController
import org.team401.taxis.diffdrive.odometry.OdometryTracker
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import kotlin.math.abs
import kotlin.math.ln
import kotlin.math.pow
import kotlin.math.tan

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */

object DrivetrainSubsystem: Subsystem(100L), IPathFollowingDiffDrive<TalonSRXVictorSensoredGearbox> by PigeonPathFollowingDiffDrive(
    TalonSRXVictorSensoredGearbox(
        Encoder(0, 1, false, CounterBase.EncodingType.k4X),
        TalonSRX(HardwareMap.CAN.drivetrainLeftMasterId),
        VictorSPX(HardwareMap.CAN.drivetrainLeftSlaveId)
    ),
    TalonSRXVictorSensoredGearbox(
        Encoder(2, 3, true, CounterBase.EncodingType.k4X),
        TalonSRX(HardwareMap.CAN.drivetrainRightMasterId),
        VictorSPX(HardwareMap.CAN.drivetrainRightSlaveId)
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

    enum class VisionContinuanceMode {
        DoNotContinue,
        ContinueRocketFront,
        ContinueHatchBack
    }

    var activeVisionContinuanceMode by LockingDelegate(VisionContinuanceMode.DoNotContinue)
    var visionContinuanceDone by LockingDelegate(false)

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
                val pitch = LeftStick.readAxis { PITCH } * -0.3 //Scale and negate to switch front
                val roll = RightStick.readAxis { ROLL } * 0.2 //Scale and negate to switch front

                /*
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

                 */

                arcade(pitch, roll)
            }
        }

        val pathFollowingKp = 0.0
        val pathFollowingKd = 0.0

        state (DriveStates.PathFollowing) {
            var lastOutputAvg = 0.0
            var lastVelAvg = 0.0
            var activeCamera: LimelightCameraEnhanced? = null
            var activeHeightMode = VisionHeightMode.NONE

            entry {
                both {
                    setDeadband(0.0)
                }

                when (activeVisionContinuanceMode) {
                    VisionContinuanceMode.DoNotContinue -> activeCamera = null
                    VisionContinuanceMode.ContinueHatchBack -> {
                        activeCamera = VisionManager.backCamera
                        activeHeightMode = VisionHeightMode.HATCH_INTAKE
                        VisionSolver.selectRegression(SuperstructureRoutines.Side.BACK, activeHeightMode, true)
                    }
                    VisionContinuanceMode.ContinueRocketFront -> {
                        activeCamera = VisionManager.frontCamera
                        activeHeightMode = VisionHeightMode.HATCH_SCORE
                        VisionSolver.selectRegression(SuperstructureRoutines.Side.FRONT, activeHeightMode, true)
                    }
                }
                activeCamera?.configForVision(activeHeightMode.pipeline)
                activeCamera?.setLedMode(LimelightCamera.LedMode.UsePipeline)
                visionContinuanceDone = false
                lastOutputAvg = 0.0
                lastVelAvg = 0.0
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

                if (pathManager.isDone) {
                    if (activeVisionContinuanceMode != VisionContinuanceMode.DoNotContinue) {
                        //Continue with vision
                        val seesTarget = activeCamera!!.seesTarget()
                        val targetAngle = activeCamera!!.entries.tx.getDouble(0.0)
                        val targetArea = activeCamera!!.getArea()
                        val correctionAngle = VisionSolver.solve(seesTarget, targetArea, targetAngle, activeCamera!!.robotToCamera)
                        val adjustment = if (!correctionAngle.isNaN()) {
                            ((correctionAngle * ControlParameters.DrivetrainParameters.visionKp) / 100.0) * 12.0
                        } else {
                            0.0
                        }
                        if (!visionContinuanceDone) {
                            if ((abs(leftVelocityActual) + abs(rightVelocityActual)) / 2.0 <= (lastVelAvg / 3.0)) {
                                visionContinuanceDone = true
                            }
                            //left.master.pidController.setReference(0.0, ControlType.kVelocity, 0, lastOutputAvg - adjustment)
                            //right.master.pidController.setReference(0.0, ControlType.kVelocity, 0, lastOutputAvg + adjustment)
                        } else {
                            //left.master.pidController.setReference(0.0, ControlType.kVelocity, 0, 0.0)
                            //right.master.pidController.setReference(0.0, ControlType.kVelocity, 0, 0.0)
                        }


                    }
                } else {
                    //Use velocity PID mode with gains all set to zero, simply to force the closed loop ramp rate (0.0)
                    //and voltage compensation using arbFF.  No actual velocity control is being done on the SPARK.
                    //left.master.pidController.setReference(0.0, ControlType.kVelocity, 0, leftOut)
                    //right.master.pidController.setReference(0.0, ControlType.kVelocity, 0, rightOut)
                    lastOutputAvg = (leftOut + rightOut) / 2.0
                    lastVelAvg = (abs(leftVelocityActual) + abs(rightVelocityActual)) / 2.0
                }


                //println("error: ${pathManager.error}")
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
            setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
            master.configContinuousCurrentLimit(40)
            master.setControlFramePeriod(ControlFrame.Control_3_General, 5)
            master.configOpenloopRamp(.25)
            master.configClosedloopRamp(0.0)

            left.inverted = false
            right.inverted = true
        }
    }

    override fun action() {
        //updateVisionDebug()

        // Detect faults

        //Motor controller reset fault


        //Sensor faults
        if (imu.state != PigeonIMU.PigeonState.Ready) {
            fault(DriveFaults.IMUFailure)
            DriverStation.reportWarning("[Fault] IMU failed!", false)
        }

        // Respond to faults


        if (isFaulted(DriveFaults.IMUFailure)) {
            if (imu.state == PigeonIMU.PigeonState.Ready) {
                clearFault(DriveFaults.IMUFailure)
                println("[Fault Cleared] IMU restored")
            }
        }

        //Insert debug println statements below:
        //println("left: ${left.getPosition()}  right: ${right.getPosition()}")

        //println(driveState.getLatestFieldToVehicle().value)
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
            setPose(Pose2d(0.0, 0.0, Rotation2d.identity()))
        }
    }
}
