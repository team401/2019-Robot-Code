package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal
import com.ctre.phoenix.motorcontrol.LimitSwitchSource
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.on
import org.snakeskin.dsl.rtAction
import org.snakeskin.dsl.stateMachine
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.*
import org.snakeskin.state.StateMachine
import org.snakeskin.subsystem.Subsystem
import org.snakeskin.utility.Ticker
import org.team401.robot2019.DriverStationDisplay
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import kotlin.math.abs
import kotlin.math.roundToInt

object ArmSubsystem: Subsystem(100L) {
    private val pivotLeftTalon = TalonSRX(HardwareMap.CAN.armPivotLeftTalonId)
    private val pivotRightTalon = TalonSRX(HardwareMap.CAN.armPivotRightTalonId)
    private val extensionTalon = TalonSRX(HardwareMap.CAN.armExtensionTalonId)

    private val pivot = CTRESmartGearbox(pivotRightTalon, pivotLeftTalon)
    private val extension = CTRESmartGearbox(extensionTalon)

    var extensionHomed by LockingDelegate(false)

    /**
     * Represents which set of gains is selected.  If true, the arm is the "vertical holding" gain set
     */
    private var isInVerticalMode = false

    private fun configureForMoveToVertical(force: Boolean = false) {
        if (force || !isInVerticalMode) {
            pivot.setPIDF(ControlParameters.ArmParameters.ArmRotationVerticalPIDF)
        }
        isInVerticalMode = true
    }

    private fun configureForMove(force: Boolean = false) {
        if (force || isInVerticalMode) {
            pivot.setPIDF(ControlParameters.ArmParameters.ArmRotationMovePIDF)
        }
        isInVerticalMode = false
    }

    /**
     * Gets the current state of the arm as measured by sensors
     */
    fun getCurrentArmState(): ArmState {
        val radius = extension.getPosition()
            .toLinearDistance(Geometry.ArmGeometry.extensionPitchRadius)
            .toInches()

        val armAngle = pivot.getPosition()
        val armVelocity = pivot.getVelocity()

        return ArmState(radius, armAngle, armVelocity)
    }

    enum class ArmPivotStates {
        EStopped, //System is stopped.  No commands sent to motors
        Holding, //System is holding its current position.
        CoordinatedControl, //System is being controlled by the motion planner.
        TuneKs, //Tunes the gain required to hold the arm
    }

    enum class ArmFaults {
        PivotEncoderFault,
        ExtensionEncoderFault,
    }

    enum class ArmExtensionStates {
        EStopped, //System is stopped.  No commands sent to motors
        Homing, //System is homing.  This means that it is slowly retracted to locate its home position
        SetHome, //Sets the home of the extension, and then goes to the safe position
        Holding, //System is holding its current position
        GoToSafe, //System moves to and holds the safe position.  In this case, it is the minimum radius
        CoordinatedControl, //System is being controlled by the motion planner.,
        TuneFf
    }

    private fun withinTolerance(target: Double, pos: Double, tolerance: Double): Boolean{
        return abs(target - pos) <= abs(tolerance)
    }

    val armPivotMachine: StateMachine<ArmPivotStates> = stateMachine {
        //rejectAllIf(*ArmPivotStates.values()) {isInState(ArmPivotStates.EStopped)}

        state (ArmPivotStates.EStopped) {
            entry {
                println("ARM PIVOT E STOP")
            }
            action {
                pivot.stop()
            }
            exit {
                println("Arm pivot operational")
            }
        }

        state (ArmPivotStates.Holding) {
            var currentPosition = 0.0
            entry {
                pivot.set(0.0)
                configureForMoveToVertical(true) //Use the less aggressive gains for holding mode
                currentPosition = pivot.getPosition().toMagEncoderTicks().value
            }

            action {
                pivot.set(ControlMode.Position, currentPosition)
            }
        }

        state (ArmPivotStates.CoordinatedControl) {
            rtAction {
                val output = SuperstructureController.output
                if (output.armAngle.toDegrees().value in 80.0..100.0) {
                    configureForMoveToVertical()
                } else {
                    configureForMove()
                }
                val angle = output.armAngle.toMagEncoderTicks().value
                val ffVoltage = output.armFeedForwardVoltage
                val ffPercent = ffVoltage / 12.0

                pivot.set(ControlMode.Position, angle, ffPercent)
            }
        }

        state (ArmPivotStates.TuneKs) {
            entry {
                SmartDashboard.setDefaultNumber("armKsSetpoint", 0.0)
            }

            action {
                val setpoint = SmartDashboard.getNumber("armKsSetpoint", 0.0)
                pivot.set(setpoint)
                val armState = getCurrentArmState()
                val outVolt = pivot.getOutputVoltage()
                val r = armState.armRadius.value
                val theta = armState.armAngle.value
                val ff = outVolt / (r * Math.cos(theta))
                println("arm ks: $ff")
            }

            exit {
                pivot.stop()
            }
        }

        disabled {
            action {
                pivot.set(0.0)
            }
        }
    }

    val armExtensionMachine: StateMachine<ArmExtensionStates> = stateMachine {
        //rejectAllIf(*ArmExtensionStates.values()){isInState(ArmExtensionStates.EStopped)}

        state (ArmExtensionStates.EStopped) {
            entry {
                println("ARM EXTENSION E STOP")
            }
            action {
                extension.stop()
            }
            exit {
                println("Arm Extension Operational")
            }
        }

        state (ArmExtensionStates.Homing) {
            val ticker = Ticker(
                {extension.getVelocity() == 0.0.RadiansPerSecond},
                ControlParameters.ArmParameters.extensionHomingTime,
                20.0.Milliseconds.toSeconds()
            )

            entry {
                ticker.reset()
                extensionHomed = false
            }

            action {
                extension.set(ControlParameters.ArmParameters.extensionHomingPower)
                ticker.check {
                    setState(ArmExtensionStates.SetHome)
                }
            }

            exit {
                extension.stop()
            }
        }

        state (ArmExtensionStates.SetHome) {
            entry {
                extension.master.selectedSensorPosition = (Geometry.ArmGeometry.armBaseLength + Geometry.ArmGeometry.armExtensionStickout)
                    .toAngularDistance(Geometry.ArmGeometry.extensionPitchRadius)
                    .toMagEncoderTicks().value.roundToInt()
                extensionHomed = true
                setState(ArmExtensionStates.GoToSafe)
            }
        }

        state (ArmExtensionStates.Holding) {
            var currentPosition = 0.0

            entry {
                currentPosition = extension.getPosition().toMagEncoderTicks().value
            }

            action {
                extension.set(ControlMode.MotionMagic, currentPosition)
            }
        }

        state (ArmExtensionStates.GoToSafe) {
            val target = (Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches)
                .toAngularDistance(Geometry.ArmGeometry.extensionPitchRadius)
                .toMagEncoderTicks().value

            action {
                extension.set(ControlMode.MotionMagic, target)
            }
        }

        state (ArmExtensionStates.CoordinatedControl) {
            rejectIf { !extensionHomed }
            rtAction {
                val output = SuperstructureController.output
                if (output.armRadius > 60.0.Inches) {
                    println("ARM EXTENSION OVERTRAVEL!  STOPPING MOTION")
                    setState(ArmExtensionStates.EStopped)
                    armPivotMachine.setState(ArmPivotStates.EStopped)
                }

                val target = output.armRadius
                    .toAngularDistance(Geometry.ArmGeometry.extensionPitchRadius)
                    .toMagEncoderTicks().value


                extension.set(ControlMode.MotionMagic, target)
            }
        }

        state (ArmExtensionStates.TuneFf) {
            var lastVel = 0

            entry {
                extension.set(.5)
            }

            action {
                lastVel = extension.master.selectedSensorVelocity
            }

            exit {
                extension.stop()
                println("Extension FF: ${.5 * 1023.0 / lastVel}")
            }
        }

        disabled {
            action {
                extension.set(0.0)
            }
        }
    }

    override fun action() {
        // Faults
        if (pivot.getSensorCollection().pulseWidthRiseToRiseUs == 0) {
            fault(ArmFaults.PivotEncoderFault)
            DriverStation.reportWarning("[Fault] Arm Pivot Encoder has failed!", false)
        }
        if (extension.getSensorCollection().pulseWidthRiseToRiseUs == 0) {
            fault(ArmFaults.ExtensionEncoderFault)
            DriverStation.reportWarning("[Fault] Arm Extension Encoder has failed!", false)
        }

        // Driver Station Shutoff
        /*
        if(DriverStationDisplay.pivotStopped.getBoolean(false)){
            armPivotMachine.setState(ArmPivotStates.EStopped)
        }else if (armPivotMachine.isInState(ArmPivotStates.EStopped) && !DriverStationDisplay.pivotStopped.getBoolean(false)
            ){
            armPivotMachine.setState(ArmPivotStates.Holding)
        }
        if(DriverStationDisplay.extensionStopped.getBoolean(false)){
            armExtensionMachine.setState(ArmExtensionStates.EStopped)
        }else if (armExtensionMachine.isInState(ArmExtensionStates.EStopped) && !DriverStationDisplay.extensionStopped.getBoolean(false)
        ){
            armExtensionMachine.setState(ArmExtensionStates.Holding)
        }
        */

        DriverStationDisplay.extensionHomed.setBoolean(extensionHomed)

        // armState = getCurrentArmState()
        //println("Arm radius: ${armState.armRadius}")
        //println("ArmState: ${armState.armAngle.toDegrees()} Raw Pos: ${pivot.getPosition().toDegrees()}")
        //println(SuperstructureController.output.armFeedForwardVoltage)
        //println(pivot.getPosition().toDegrees())
        //println(extension.getPosition().toLinearDistance(Geometry.ArmGeometry.extensionPitchRadius))
        //println("Arm Position : ${ArmKinematics.forward(getCurrentArmState())}")
        //println(SuperstructureController.output.visionHeightMode)
        //println(pivotRightTalon.sensorCollection.pulseWidthPosition)

    }

    override fun setup() {
        pivotLeftTalon.inverted = true
        pivotRightTalon.inverted = false

        pivot.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        pivot.setForwardLimitSwitch(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled)
        pivot.setReverseLimitSwitch(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled)
        pivot.setCurrentLimit(30.0, 0.0, 0.0.Seconds)
        pivot.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
        if (ControlParameters.ArmParameters.armEncoderPhase) {
            pivot.master.selectedSensorPosition = (Math.abs(pivot.getSensorCollection().pulseWidthPosition % 4096.0).roundToInt() - ControlParameters.ArmParameters.armEncoderValueAtVertical - 1024)

        } else {
            pivot.master.selectedSensorPosition = (Math.abs(pivot.getSensorCollection().pulseWidthPosition % 4096.0).roundToInt() - ControlParameters.ArmParameters.armEncoderValueAtVertical + 1024)
        }
        pivot.setSensorPhase(ControlParameters.ArmParameters.armEncoderPhase)
        configureForMoveToVertical(true)

        extension.inverted = false
        extension.setSensorPhase(true)
        extension.setForwardLimitSwitch(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled)
        extension.setReverseLimitSwitch(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled)

        extension.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        extension.setCurrentLimit(30.0, 0.0, 0.0.Seconds)
        extension.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)

        val nativeVelocity = ControlParameters.ArmParameters.extensionVelocity
            .toAngularVelocity(Geometry.ArmGeometry.extensionPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()

        val nativeAcceleration = ControlParameters.ArmParameters.extensionAcceleration
            .toAngularVelocity(Geometry.ArmGeometry.extensionPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND

        extension.master.configMotionCruiseVelocity(nativeVelocity)
        extension.master.configMotionAcceleration(nativeAcceleration)

        extension.setPIDF(ControlParameters.ArmParameters.ArmExtensionPIDF)

        on (Events.TELEOP_ENABLED) {
            if (!extensionHomed) {
                armExtensionMachine.setState(ArmExtensionStates.Homing)
            } else {
                val currentRadius = getCurrentArmState().armRadius
                if (currentRadius <= Geometry.ArmGeometry.minSafeArmLength) {
                    armExtensionMachine.setState(ArmExtensionStates.GoToSafe)
                } else {
                    armExtensionMachine.setState(ArmExtensionStates.Holding)
                }
            }
            armPivotMachine.setState(ArmPivotStates.Holding)
        }
    }
}