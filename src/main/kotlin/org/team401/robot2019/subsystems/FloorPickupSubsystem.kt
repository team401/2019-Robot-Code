package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import edu.wpi.first.wpilibj.PowerDistributionPanel
import edu.wpi.first.wpilibj.Solenoid
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.Milliseconds
import org.snakeskin.measure.Seconds
import org.snakeskin.utility.Ticker
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.HardwareMap

/**
 * @author Cameron Earle
 * @version 2/10/2019
 *
 */
object FloorPickupSubsystem: Subsystem(100L) {
    private val piston = Solenoid(HardwareMap.Pneumatics.floorPickupSolenoid)
    private val wheels = VictorSPX(HardwareMap.CAN.floorPickupWheelsVictorId)

    enum class PickupStates {
        EStopped,
        Stowed,
        Deployed
    }

    enum class WheelsStates {
        EStopped,
        Idle,
        Intake,
        Eject
    }

    val pickupMachine: StateMachine<PickupStates> = commandMachine(
        stateMap(
            PickupStates.EStopped to false,
            PickupStates.Stowed to false,
            PickupStates.Deployed to true
        )
    ) {
        piston.set(value)
    }

    val wheelsMachine: StateMachine<WheelsStates> = commandMachine(
        stateMap(
            WheelsStates.EStopped to 0.0,
            WheelsStates.Idle to 0.0,
            //WheelsStates.Intake to ControlParameters.FloorPickupParameters.intakeSpeed,
            WheelsStates.Eject to ControlParameters.FloorPickupParameters.ejectSpeed
        ),
        { wheels.set(ControlMode.PercentOutput, value) },
        {
            state(FloorPickupSubsystem.WheelsStates.Intake){
                val currentTimeout = Ticker(
                    { PowerDistributionPanel().getCurrent(10) > 4.0},
                    0.2.Seconds,
                    20.0.Milliseconds.toSeconds()
                )
                entry {

                    currentTimeout.reset()
                    wheels.set(ControlMode.PercentOutput, ControlParameters.FloorPickupParameters.intakeSpeed)
                }
                action {
                    println("Current : ${PowerDistributionPanel().getCurrent(10)}")
                    currentTimeout.check{
                        setState(FloorPickupSubsystem.WheelsStates.Idle)
                        pickupMachine.setState(FloorPickupSubsystem.PickupStates.Stowed)
                    }
                }
            }
        }
    )

    override fun action() {
        /*
        if (DriverStationDisplay.floorPickupStopped.getBoolean(false)) {
            pickupMachine.setState(PickupStates.EStopped)
            wheelsMachine.setState(WheelsStates.EStopped)
        } else if (pickupMachine.isInState(PickupStates.EStopped) || wheelsMachine.isInState(WheelsStates.EStopped)
        && !DriverStationDisplay.floorPickupStopped.getBoolean(false)) {
            pickupMachine.setState(PickupStates.Stowed)
            wheelsMachine.setState(WheelsStates.Idle)
        }
        */
    }
    override fun setup() {
        wheels.inverted = false
        wheels.setNeutralMode(NeutralMode.Brake)

        on (Events.ENABLED) {
            //pickupMachine.setState(PickupStates.Stowed)
            wheelsMachine.setState(WheelsStates.Idle)
        }
    }
}