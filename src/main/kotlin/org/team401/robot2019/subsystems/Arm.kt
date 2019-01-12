package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.snakeskin.dsl.on
import org.snakeskin.dsl.stateMachine
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.state.StateMachine
import org.snakeskin.subsystem.Subsystem
import org.snakeskin.units.MagEncoderTicks
import org.snakeskin.units.MagEncoderTicksPer100Ms
import org.snakeskin.units.RevolutionsPerSecond
import org.team401.robot2019.Gamepad
import org.team401.robot2019.config.ControlParameters.ArmParameters
import kotlin.math.abs

object PrototypeArm: Subsystem() {

    private val rotationMotor = TalonSRX(0)
    private val extensionMotor = TalonSRX(1)

    private var homed by LockingDelegate(false)

    override fun setup() {
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 1, 0)
        rotationMotor.configPeakCurrentLimit(10)
        rotationMotor.configPeakCurrentDuration(100)
        rotationMotor.

        // TODO Remember - Chain reduction is 16:72

        extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
        extensionMotor.configPeakCurrentLimit(10)
        extensionMotor.configPeakCurrentDuration(100)

        //TODO Switch these!!
        on(Events.TELEOP_ENABLED){
            if(homed){
                armMachine.setState(ArmStates.MANUAL_CONTROL)
            }else {
                armMachine.setState(ArmStates.HOMING)
            }
        }
        on(Events.DISABLED){
            armMachine.setState(ArmStates.TESTING)
        }

    }
    enum class ArmStates{
        E_STOPPED, HOMING, MANUAL_CONTROL, BETTER_CONTROL, HOLDING, TESTING
    }
    enum class ExtensionSates{
        E_STOPPED, MANUAL_CONTROL
    }

    private fun withinTolerance(target: Double, pos: Double, tolerance: Double): Boolean{
        return abs(target - pos) <= abs(tolerance)
    }

    override fun action() {
        println("Position: ${rotationMotor.selectedSensorPosition.MagEncoderTicks.value} encoder ticks")
        println("State ${armMachine.getState()}")
    }

    val armMachine: StateMachine<ArmStates> = stateMachine {
        rejectAllIf(*ArmStates.values()){isInState(ArmStates.E_STOPPED)}

        state(ArmStates.E_STOPPED){
            entry {
                rotationMotor.set(ControlMode.PercentOutput, 0.0)
            }
        }

        state(ArmStates.MANUAL_CONTROL){
            action {
                // Slowed down to not kill the arm for testing
                var input = Gamepad.readAxis { LEFT_Y }
                println("Manual control $input")
                if (withinTolerance(0.0, input, 0.05)){
                    val position = rotationMotor.selectedSensorPosition.MagEncoderTicks
                    rotationMotor.set(ControlMode.MotionMagic, position.value)
                }else {
                    rotationMotor.set(ControlMode.PercentOutput, 0.25 * input)
                }
            }
        }

        state(ArmStates.BETTER_CONTROL){
            entry {
                println("Please stand by for a better controller")
            }

        }

        state(ArmStates.TESTING){
            action {
                println("Position: ${rotationMotor.selectedSensorPosition.MagEncoderTicks.value} encoder ticks")
            }
        }

        state(ArmStates.HOLDING){
            entry {
                val pos = rotationMotor.selectedSensorPosition.MagEncoderTicks
                rotationMotor.set(ControlMode.MotionMagic, pos.value)
            }
        }

        state(ArmStates.HOMING){
            var velocityCounter = 0
            entry {
                rotationMotor.configMotionAcceleration(ArmParameters.MAX_ACCELERATION)
                rotationMotor.configMotionCruiseVelocity(ArmParameters.MAX_VELOCITY)
                rotationMotor.selectProfileSlot(1, 1)
                //rotationMotor.set(ControlMode.Velocity, 0.05.RevolutionsPerSecond.toUnit(MagEncoderTicksPer100Ms).value)
            }
            action{
                println("Homing action")
                rotationMotor.set(ControlMode.Velocity, 0.1.RevolutionsPerSecond.toUnit(MagEncoderTicksPer100Ms).value)

                var velocity = rotationMotor.selectedSensorVelocity.MagEncoderTicksPer100Ms
                if(velocity.value < ArmParameters.HOMING_VELOCITY.value){
                    velocityCounter++
                }else{
                    velocityCounter = 0
                }

                if(velocityCounter > 10){
                    rotationMotor.set(ControlMode.PercentOutput, 0.0)
                    homed = true
                    println("Homed position: ${rotationMotor.selectedSensorPosition} ticks")
                    //TODO Find a home (constant)
                    //rotationMotor.selectedSensorPosition =
                    setState(ArmStates.MANUAL_CONTROL)
                }
            }
        }
    }

    val extentionMachine: StateMachine<ExtensionSates> = stateMachine {
        rejectAllIf(*ExtensionSates.values()){isInState(ExtensionSates.E_STOPPED)}

        state(ExtensionSates.E_STOPPED){
            entry {
                extensionMotor.set(ControlMode.PercentOutput, 0.0)
            }
        }
    }
}