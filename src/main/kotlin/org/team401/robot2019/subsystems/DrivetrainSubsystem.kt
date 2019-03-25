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
import org.snakeskin.component.impl.SparkMaxCTRESensoredGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.*
import org.snakeskin.utility.CheesyDriveController
import org.team401.robot2019.DriverStationDisplay
import org.team401.robot2019.LeftStick
import org.team401.robot2019.RightStick
import org.team401.robot2019.RobotEvents
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.config.Physics
import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.robot2019.control.vision.VisionKinematics
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.control.vision.VisionState
import org.team401.robot2019.util.PathReader
import org.team401.robot2019.util.TrajectoryPath
import org.team401.taxis.diffdrive.component.IPathFollowingDiffDrive
import org.team401.taxis.diffdrive.component.impl.PigeonPathFollowingDiffDrive
import org.team401.taxis.diffdrive.control.NonlinearFeedbackPathController
import org.team401.taxis.diffdrive.odometry.OdometryTracker
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.trajectory.timing.CentripetalAccelerationConstraint
import org.team401.taxis.trajectory.timing.TimedState

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */

object DrivetrainSubsystem: Subsystem(500L), IPathFollowingDiffDrive<SparkMaxCTRESensoredGearbox<TalonSRX>> by PigeonPathFollowingDiffDrive(
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

    var activeFieldToGoal by LockingDelegate(Pose2d.identity())

    /**
     * Shifts the drivetrain to the selected gear.  The available shifter states are available in ShifterStates.
     */
    fun shift(state: Boolean) {
        shifter.set(state)
    }

    enum class DriveStates(val requiresSensors: Boolean = false) {
        EStopped,
        DisabledForFault,
        OpenLoopOperatorControl,
        PathFollowing(true),
        ClimbPull,
        ClimbStop,
        ClimbReposition,
        HatchAlignFront
    }

    enum class DriveFaults {
        MotorControllerReset,
        LeftEncoderFailure,
        RightEncoderFailure,
        IMUFailure
    }

    private val cheesyController = CheesyDriveController(ControlParameters.DrivetrainCheesyDriveParameters)

    val stateEstimator = OdometryTracker(this)

    const val gearRatioHigh = (50.0/28.0) * (64.0 / 15.0)

    val driveMachine: StateMachine<DriveStates> = stateMachine {
        rejectAllIf(*DriveStates.values()){isInState(DriveStates.EStopped)}

        state(DriveStates.EStopped){
            entry {
                println("DRIVETRAIN E STOP")
                stop()
            }
        }
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

                setPose(Pose2d(5.5 * 12, 17.25 * 12, Rotation2d.fromDegrees(0.0)))

                VisionState.reset()
                pathManager.reset()
                pathManager.setTrajectory(TrajectoryIterator(TimedView(
                    pathManager.generateTrajectory(
                        false,
                        listOf(
                            Pose2d(5.5 * 12, 17.25 * 12, Rotation2d.fromDegrees(0.0)),
                            Pose2d(19.5 * 12, 18.5 * 12, Rotation2d.fromDegrees(15.0)),
                            Pose2d(20.75 * 12, 25.0 * 12, Rotation2d.fromDegrees(145.0))
                        ),
                        listOf(CentripetalAccelerationConstraint(130.0)),
                        4.0 * 12,
                        4.0 * 12,
                        9.0
                    )
                )))
            }

            rtAction {
                //TODO return this to the pose from the regular driveState
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

                println("error: ${pathManager.error}")
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

        state(DriveStates.HatchAlignFront) {
            var vLoc = false
            var tGen = false
            var fieldToGoal = Pose2d.identity()
            var startPose = Pose2d.identity()
            var startVelocity = 0.0.InchesPerSecond
            var trajectory: TrajectoryIterator<TimedState<Pose2dWithCurvature>>

            /*
            rejectIf {
                SuperstructureController.output.wristTool != WristMotionPlanner.Tool.HatchPanelTool
            }
            */

            entry {
                setPose(Pose2d.identity())
                vLoc = false
                tGen = false
                VisionManager.frontCamera.configForVision(1)
                VisionManager.frontCamera.setLedMode(LimelightCamera.LedMode.UsePipeline)
                VisionManager.frontCamera.resetFrame()
                Thread.sleep(100) //Give the camera some time to enter the state
            }

            rtAction {
                if (!vLoc) {
                    //Look for target
                    val frame = VisionManager.frontCamera.frame
                    val poseAtCapture = driveState.getFieldToVehicle(frame.timeCaptured.value)
                    startPose = driveState.getFieldToVehicle(time)
                    startVelocity = (left.getVelocity().toLinearVelocity(Geometry.DrivetrainGeometry.wheelRadius) +
                            right.getVelocity().toLinearVelocity(Geometry.DrivetrainGeometry.wheelRadius)) / 2.0.Unitless
                    if (frame.hasTarget) {
                        val fieldToGoalCaptured = VisionKinematics.solveFieldToGoal(
                            poseAtCapture,
                            Geometry.VisionGeometry.robotToFrontCamera,
                            frame.toPose2d()
                        )

                        println("Start pose: $startPose")
                        println("Field to goal captured: $fieldToGoalCaptured")
                        fieldToGoal = VisionKinematics.solveLatencyCorrection(poseAtCapture, startPose, fieldToGoalCaptured)
                        println("Field to goal latency corrected: $fieldToGoal")
                        send(RobotEvents.VLoc)
                        vLoc = true
                    }
                } else {
                    //Have target, trajectory time
                    if (!tGen) {
                        //Generate the trajectory
                        val goal = fieldToGoal.transformBy(Pose2d(-38.0, 0.0, Rotation2d.identity())) //Account for arm length
                        val straightPart = goal.transformBy(Pose2d(-6.0, 0.0, Rotation2d.identity())) //Add some points in the profile to drive straight

                        val maxVelocity = 6.0 * 12.0
                        val maxAcceleration = 4.0 * 12.0
                        val maxVoltage = 9.0

                        PathReader.outputToPathViewer(
                            TrajectoryPath(
                                false,
                                listOf(startPose, straightPart, goal),
                                maxVelocity,
                                maxAcceleration,
                                maxVoltage
                            ), "Vision Alignment"
                        )

                        trajectory = TrajectoryIterator(
                            TimedView(
                                pathManager.generateTrajectory(
                                    false,
                                    listOf(startPose, goal),
                                    listOf(),
                                    startVelocity.value,
                                    0.0,
                                    maxVelocity,
                                    maxAcceleration,
                                    maxVoltage
                                )
                            )
                        )

                        println("Driving: ${listOf(startPose, goal)}")
                        println("Straight Part: $straightPart")


                        //Run the trajectory
                        pathManager.reset()
                        pathManager.setTrajectory(
                            trajectory
                        )

                        tGen = true
                        setState(DriveStates.PathFollowing)
                    }
                }
            }

            exit {
                //VisionManager.frontCamera.configForVision(1)
                //VisionManager.frontCamera.setLedMode(LimelightCamera.LedMode.Off)
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
        rightTalon.setSensorPhase(false)

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
        if (left.ctreController.sensorCollection.pulseWidthRiseToRiseUs == 0) {
            fault(DriveFaults.LeftEncoderFailure)
            DriverStation.reportWarning("[Fault] Left encoder failed!", false)
        }
        if (right.ctreController.sensorCollection.pulseWidthRiseToRiseUs == 0) {
            fault(DriveFaults.RightEncoderFailure)
            DriverStation.reportWarning("[Fault] Right encoder failed!", false)
        }
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

        //Sensor faults
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
        if (isFaulted(DriveFaults.IMUFailure)) {
            if (imu.state == PigeonIMU.PigeonState.Ready) {
                clearFault(DriveFaults.IMUFailure)
                println("[Fault Cleared] IMU restored")
            }
        }

        // Driver Station Shutoff
        if (DriverStationDisplay.driveStopped.getBoolean(false)) {
            driveMachine.setState(DriveStates.EStopped)
        }

        //Insert debug println statements below:
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
