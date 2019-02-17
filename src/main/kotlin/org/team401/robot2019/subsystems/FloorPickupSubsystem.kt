package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Solenoid
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.HardwareMap

/**
 * @author Cameron Earle
 * @version 2/10/2019
 *
 */
object FloorPickupSubsystem: Subsystem() {
    //private val piston = Solenoid(HardwareMap.FloorPickup.solenoidId)
    private val wheels = TalonSRX(HardwareMap.FloorPickup.intakeWheelsTalonId)

    enum class PickupStates {
        Stowed,
        Deployed
    }

    enum class WheelsStates {
        Idle,
        Intake,
        Eject
    }

    /*
    val pickupMachine: StateMachine<PickupStates> = commandMachine(
        stateMap(
            PickupStates.Stowed to false,
            PickupStates.Deployed to true
        )
    ) {
        //piston.set(value)
    }
    */


    val wheelsMachine: StateMachine<WheelsStates> = commandMachine(
        stateMap(
            WheelsStates.Idle to 0.0,
            WheelsStates.Intake to ControlParameters.FloorPickupParameters.intakeSpeed,
            WheelsStates.Eject to ControlParameters.FloorPickupParameters.ejectSpeed
        )
    ) {
        wheels.set(ControlMode.PercentOutput, value)
    }


    /*
    val wheelsMachine: StateMachine<WheelsStates> = stateMachine {
        state(WheelsStates.Idle) {
            action {
                wheels.set(ControlMode.PercentOutput, 0.0)
            }
        }

        state(WheelsStates.Eject) {
            action {
                wheels.set(ControlMode.PercentOutput, ControlParameters.FloorPickupParameters.ejectSpeed)
            }
        }

        state(WheelsStates.Intake) {
            action {
                wheels.set(ControlMode.PercentOutput, ControlParameters.FloorPickupParameters.intakeSpeed)
            }
        }
    }
    */

    override fun setup() {
        wheels.inverted = false
        wheels.setNeutralMode(NeutralMode.Brake)

        on (Events.ENABLED) {
            //pickupMachine.setState(PickupStates.Stowed)
            wheelsMachine.setState(WheelsStates.Idle)
        }
    }
}