package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.snakeskin.dsl.stateMachine
import org.snakeskin.state.StateMachine
import org.snakeskin.subsystem.Subsystem
import org.team401.robot2019.Gamepad
import org.team401.robot2019.config.ControlParameters.ArmParameters
import kotlin.math.abs

object PrototypeArm: Subsystem() {

    private val rotationMotor = TalonSRX(0)
    private val extensionMotor = TalonSRX(1)

    override fun setup() {
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute)
        rotationMotor.configPeakCurrentLimit(10)
        rotationMotor.configPeakCurrentDuration(100)



        extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute)
        extensionMotor.configPeakCurrentLimit(10)
        extensionMotor.configPeakCurrentDuration(100)
    }
    enum class ArmStates{
        E_STOPPED, HOMING, MANUAL_CONTROL, BETTER_CONTROL, HOLDING
    }
    enum class ExtensionSates{
        E_STOPPED, MANUAL_CONTROL
    }

    fun withinTolerance(target: Double, pos: Double, tolerance: Double): Boolean{
        return abs(target - pos) <= abs(tolerance)
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
                var input = Gamepad.readAxis { LEFT_X }
                if (withinTolerance(0.0, input, 0.05)){
                    val position = rotationMotor.selectedSensorPosition.toDouble()
                    rotationMotor.set(ControlMode.MotionMagic, position)
                }else {
                    rotationMotor.set(ControlMode.PercentOutput, 0.25 * input)
                }

            }
        }

        state(ArmStates.BETTER_CONTROL){
            entry {
                println("Please stand by for a real controller")
            }

        }

        state(ArmStates.HOLDING){
            entry {
                val position = rotationMotor.selectedSensorPosition.toDouble()
                rotationMotor.set(ControlMode.MotionMagic, position)
            }
        }
        
        state(ArmStates.HOMING){
            var velocityCounter = 0
            entry {
                rotationMotor.configMotionAcceleration(ArmParameters.MAX_ACCELERATION)
                rotationMotor.configMotionCruiseVelocity(ArmParameters.MAX_VELOCITY)
                rotationMotor.set(ControlMode.PercentOutput, 0.25)
            }
            action{
                if(rotationMotor.selectedSensorVelocity < ArmParameters.HOMING_VELOCITY){
                    velocityCounter++
                }else{
                    velocityCounter = 0
                }

                if(velocityCounter > 20){
                    rotationMotor.set(ControlMode.PercentOutput, 0.0)
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