package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.snakeskin.component.TalonPigeonIMU
import org.snakeskin.dsl.StateMachine
import org.snakeskin.dsl.on
import org.snakeskin.dsl.stateMachine
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.subsystem.Subsystem
import org.snakeskin.units.AngularDistanceUnitCTREMagEncoder
import org.snakeskin.units.Degrees
import org.team401.robot2019.config.HardwareMap

/**
 * @author Cameron Earle
 * @version 1/10/2019
 *
 */
object ArmTestSubsystem: Subsystem() {
    private val talon = TalonSRX(9)
    private val pigeon = TalonPigeonIMU(HardwareMap.Drivetrain.pigeonImuId)

    private val pigeonYpr = DoubleArray(3) //Roll is target axis

    var armSetpoint by LockingDelegate(0.0)

    val armMachine: StateMachine<String> = stateMachine {
        state("calc_f") {
            action {
                talon.set(ControlMode.PercentOutput, 1.0)
                println(talon.selectedSensorVelocity)
            }
        }

        state("control_setpoint") {
            entry {
                talon.configMotionCruiseVelocity(4000)
                talon.configMotionAcceleration(4000 * 4)
            }
            action {
                talon.set(ControlMode.MotionMagic, armSetpoint)
            }
        }

        state("yaw_control") {
            entry {
                talon.configMotionCruiseVelocity(4000)
                talon.configMotionAcceleration(4000 * 4)
            }
            action {
                pigeon.getYawPitchRoll(pigeonYpr)
                talon.set(ControlMode.MotionMagic, pigeonYpr[1].Degrees.toUnit(AngularDistanceUnitCTREMagEncoder).value)
            }
        }
    }

    override fun setup() {
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
        talon.selectedSensorPosition = 0
        on (Events.TELEOP_ENABLED) {
            //armMachine.setState("yaw_control")
        }
    }

    override fun action() {
        pigeon.getYawPitchRoll(pigeonYpr)
        println(pigeonYpr[0])
    }
}