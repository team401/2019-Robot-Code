package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.snakeskin.dsl.stateMachine
import org.snakeskin.state.StateMachine
import org.snakeskin.subsystem.Subsystem

object PrototypeArm: Subsystem() {

    private val rotationMotor = TalonSRX(0)
    private val extensionMotor = TalonSRX(1)

    override fun setup() {
        rotationMotor.configPeakCurrentLimit(10)
        rotationMotor.configPeakCurrentDuration(100)

        extensionMotor.configPeakCurrentLimit(10)
        extensionMotor.configPeakCurrentDuration(100)
    }
    enum class ArmStates{
        E_STOPPED, MANUAL_CONTROL
    }
    enum class ExtensionSates{
        E_STOPPED, MANUAL_CONTROL
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