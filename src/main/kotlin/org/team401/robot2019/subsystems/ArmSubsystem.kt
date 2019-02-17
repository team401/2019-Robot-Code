package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.rtAction
import org.snakeskin.dsl.stateMachine
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.*
import org.snakeskin.state.StateMachine
import org.snakeskin.subsystem.Subsystem
import org.snakeskin.utility.Ticker
import org.team401.robot2019.Gamepad
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.*
import kotlin.math.abs
import kotlin.math.roundToInt

object ArmSubsystem: Subsystem() {
    private val pivotLeftTalon = TalonSRX(HardwareMap.Arm.pivotLeftTalonId)
    private val pivotRightTalon = TalonSRX(HardwareMap.Arm.pivotRightTalonId)
    private val extensionTalon = TalonSRX(HardwareMap.Arm.extensionTalonId)

    private val pivot = CTRESmartGearbox(pivotLeftTalon, pivotRightTalon)
    private val extension = CTRESmartGearbox(extensionTalon)

    var extensionHomed by LockingDelegate(false)

    /**
     * Gets the current state of the arm as measured by sensors
     */
    fun getCurrentArmState(): ArmState {
        val extensionLength = extension.getPosition()
            .toLinearDistance(Geometry.ArmGeometry.extensionAngularToLinearRadius)
            .toInches()
        val radius = extensionLength + Geometry.ArmGeometry.armBaseLength

        val armAngle = pivot.getPosition()
        val armVelocity = pivot.getVelocity()

        return ArmState(radius, armAngle, armVelocity)
    }

    enum class ArmPivotStates {
        EStopped, //System is stopped.  No commands sent to motors
        Holding, //System is holding its current position.
        CoordinatedControl, //System is being controlled by the motion planner.
    }

    enum class ArmExtensionStates {
        EStopped, //System is stopped.  No commands sent to motors
        Homing, //System is homing.  This means that it is slowly retracted to locate its home position
        Holding, //System is holding its current position
        GoToSafe, //System moves to and holds the safe position.  In this case, it is the minimum radius
        CoordinatedControl, //System is being controlled by the motion planner.
    }

    private fun withinTolerance(target: Double, pos: Double, tolerance: Double): Boolean{
        return abs(target - pos) <= abs(tolerance)
    }

    val armPivotMachine: StateMachine<ArmPivotStates> = stateMachine {
        rejectAllIf(*ArmPivotStates.values()) {isInState(ArmPivotStates.EStopped)}

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
    }

    val armExtensionMachine: StateMachine<ArmExtensionStates> = stateMachine {
        state (ArmExtensionStates.EStopped) {
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
                    extension.setPosition(
                        Geometry.ArmGeometry.armExtensionStickout
                            .toAngularDistance(Geometry.ArmGeometry.extensionAngularToLinearRadius)
                            .toRadians()
                    )
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
                extension.set(ControlMode.Position, currentPosition)
            }
        }

        state (ArmExtensionStates.GoToSafe) {
            entry {
                val target = (Geometry.ArmGeometry.minSafeWristRotationHeight - Geometry.ArmGeometry.armBaseLength)
                    .toAngularDistance(Geometry.ArmGeometry.extensionAngularToLinearRadius)
                    .toMagEncoderTicks().value

                extension.set(ControlMode.MotionMagic, target)
            }
        }

        state (ArmExtensionStates.CoordinatedControl) {
            rtAction {
                val output = SuperstructureController.output
                val target = (output.armRadius - Geometry.ArmGeometry.armBaseLength)
                    .toAngularDistance(Geometry.ArmGeometry.extensionAngularToLinearRadius)
                    .toMagEncoderTicks().value

                extension.set(ControlMode.MotionMagic, target)
            }
        }
    }

    override fun action() {
        println(extension.getPosition().toLinearDistance(Geometry.ArmGeometry.extensionAngularToLinearRadius))
    }

    override fun setup() {
        extension.inverted = false

        extension.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        extension.setCurrentLimit(30.0, 0.0, 0.0.Seconds)
        extension.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)

        val nativeVelocity = ControlParameters.ArmParameters.extensionVelocity
            .toAngularVelocity(Geometry.ArmGeometry.extensionAngularToLinearRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()

        val nativeAcceleration = ControlParameters.ArmParameters.extensionAcceleration
            .toAngularVelocity(Geometry.ArmGeometry.extensionAngularToLinearRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND

        extension.master.configMotionCruiseVelocity(nativeVelocity)
        extension.master.configMotionAcceleration(nativeAcceleration)
    }
}