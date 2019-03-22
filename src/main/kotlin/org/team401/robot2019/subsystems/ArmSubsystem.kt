package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal
import com.ctre.phoenix.motorcontrol.LimitSwitchSource
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.rtAction
import org.snakeskin.dsl.stateMachine
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.*
import org.snakeskin.state.StateMachine
import org.snakeskin.subsystem.Subsystem
import org.snakeskin.utility.Ticker
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.*
import kotlin.math.abs
import kotlin.math.roundToInt

import org.snakeskin.dsl.*
import org.team401.robot2019.DriverStationDisplay

object ArmSubsystem: Subsystem() {
    private val pivotLeftTalon = TalonSRX(HardwareMap.Arm.pivotLeftTalonId)
    private val pivotRightTalon = TalonSRX(HardwareMap.Arm.pivotRightTalonId)
    private val extensionTalon = TalonSRX(HardwareMap.Arm.extensionTalonId)

    private val pivot = CTRESmartGearbox(pivotRightTalon, pivotLeftTalon)
    private val extension = CTRESmartGearbox(extensionTalon)

    var extensionHomed by LockingDelegate(false)

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

    enum class ArmExtensionStates {
        EStopped, //System is stopped.  No commands sent to motors
        Homing, //System is homing.  This means that it is slowly retracted to locate its home position
        Holding, //System is holding its current position
        GoToSafe, //System moves to and holds the safe position.  In this case, it is the minimum radius
        CoordinatedControl, //System is being controlled by the motion planner.,
        TuneFf,
        GoTo1Foot,
        GoTo1Point5Foot,
        GoTo2Foot,
        GoTo1Inch,
    }

    private fun withinTolerance(target: Double, pos: Double, tolerance: Double): Boolean{
        return abs(target - pos) <= abs(tolerance)
    }

    val armPivotMachine: StateMachine<ArmPivotStates> = stateMachine {
        //rejectAllIf(*ArmPivotStates.values()) {isInState(ArmPivotStates.EStopped)}

        state (ArmPivotStates.EStopped) {
            action {
                pivot.stop()
            }
        }

        state (ArmPivotStates.Holding) {
            entry {
                val currentPosition = pivot.getPosition().toMagEncoderTicks().value
                pivot.set(ControlMode.Position, currentPosition)
            }
        }

        state (ArmPivotStates.CoordinatedControl) {
            rtAction {
                val output = SuperstructureController.output
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
    }

    val armExtensionMachine: StateMachine<ArmExtensionStates> = stateMachine {
        state (ArmExtensionStates.EStopped) {
            entry {
                DriverStationDisplay.armStopped.setBoolean(true)
            }
            action {
                extension.stop()
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
                extension.set(ControlParameters.ArmParameters.extensionHomingPower)
            }

            action {
                ticker.check {
                    extension.master.selectedSensorPosition = (Geometry.ArmGeometry.armBaseLength + Geometry.ArmGeometry.armExtensionStickout)
                        .toAngularDistance(Geometry.ArmGeometry.extensionPitchRadius)
                        .toMagEncoderTicks().value.roundToInt()
                    extensionHomed = true
                    setState(ArmExtensionStates.GoToSafe)
                }
            }

            exit {
                extension.stop()
            }
        }

        state (ArmExtensionStates.Holding) {
            entry {
                val currentPosition = extension.getPosition().toMagEncoderTicks().value
                extension.set(ControlMode.MotionMagic, currentPosition)
            }
        }

        state (ArmExtensionStates.GoToSafe) {
            entry {
                val target = (Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches)
                    .toAngularDistance(Geometry.ArmGeometry.extensionPitchRadius)
                    .toMagEncoderTicks().value

                extension.set(ControlMode.MotionMagic, target)
            }
        }

        state (ArmExtensionStates.CoordinatedControl) {
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

        state (ArmExtensionStates.GoTo1Foot) {
            entry {
                val target = (Geometry.ArmGeometry.armBaseLength + 1.0.Feet).toAngularDistance(
                    Geometry.ArmGeometry.extensionPitchRadius
                ).toMagEncoderTicks().value
                extension.set(ControlMode.MotionMagic, target)
            }
        }

        state (ArmExtensionStates.GoTo1Point5Foot) {
            entry {
                val target = (Geometry.ArmGeometry.armBaseLength + 1.5.Feet).toAngularDistance(
                    Geometry.ArmGeometry.extensionPitchRadius
                ).toMagEncoderTicks().value
                extension.set(ControlMode.MotionMagic, target)
            }
        }

        state (ArmExtensionStates.GoTo2Foot) {
            entry {
                val target = (Geometry.ArmGeometry.armBaseLength + 2.0.Feet).toAngularDistance(
                    Geometry.ArmGeometry.extensionPitchRadius
                ).toMagEncoderTicks().value
                extension.set(ControlMode.MotionMagic, target)
            }
        }

        state (ArmExtensionStates.GoTo1Inch) {
            entry {
                val target = (Geometry.ArmGeometry.armBaseLength + 1.0.Inches).toAngularDistance(
                    Geometry.ArmGeometry.extensionPitchRadius
                ).toMagEncoderTicks().value
                extension.set(ControlMode.MotionMagic, target)
            }
        }

        disabled {
            action {
                extension.set(0.0)
            }
        }
    }

    override fun action() {
        val armState = getCurrentArmState()
        //println(ArmKinematics.forward(armState))
        //println(SuperstructureController.output.armFeedForwardVoltage)
        //println(pivot.getPosition().toDegrees())
        //println(extension.getPosition().toLinearDistance(Geometry.ArmGeometry.extensionPitchRadius))
    }

    override fun setup() {
        pivotLeftTalon.inverted = true
        pivotRightTalon.inverted = false

        pivot.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        pivot.setForwardLimitSwitch(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled)
        pivot.setReverseLimitSwitch(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled)
        pivot.setCurrentLimit(30.0, 0.0, 0.0.Seconds)
        pivot.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
        val multiplier = if (ControlParameters.ArmParameters.armEncoderPhase) -1 else 1
        pivot.master.selectedSensorPosition = multiplier * (Math.abs(pivot.getSensorCollection().pulseWidthPosition % 4096.0).roundToInt() - ControlParameters.ArmParameters.armEncoderValueAtVertical + 1024)
        pivot.setSensorPhase(ControlParameters.ArmParameters.armEncoderPhase)
        pivot.setPIDF(ControlParameters.ArmParameters.ArmRotationPIDF)

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

        on (Events.ENABLED) {
            if (!extensionHomed) {
                armExtensionMachine.setState(ArmExtensionStates.Homing)
            } else {
                armExtensionMachine.setState(ArmExtensionStates.GoToSafe)
            }
            armPivotMachine.setState(ArmPivotStates.Holding)
        }
    }
}