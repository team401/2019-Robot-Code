package org.team401.robot2019.subsystems.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.snakeskin.dsl.on
import org.snakeskin.dsl.stateMachine
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.state.StateMachine
import org.snakeskin.subsystem.Subsystem
import org.snakeskin.units.*
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.robot2019.Gamepad
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.ControlParameters.ArmParameters
import org.team401.robot2019.controllers.ArmKinematics
import kotlin.math.abs

object PrototypeArm: Subsystem() {

    private val rotationMotor = TalonSRX(20)
    private val extensionMotor = TalonSRX(21)

    private var position = rotationMotor.selectedSensorPosition.Radians
    private var velocity = rotationMotor.selectedSensorVelocity.RadiansPerSecond
    private var armLength = ControlParameters.ArmParameters.MIN_ARM_LENGTH

    private var homed by LockingDelegate(false)

    override fun setup() {
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 1, 0)
        rotationMotor.configPeakCurrentLimit(10)
        rotationMotor.configPeakCurrentDuration(100)
        rotationMotor.configClosedLoopPeakOutput(0, 0.25)
        rotationMotor.configClosedLoopPeakOutput(1, 0.25)


        // TODO Remember - Chain reduction is 16:72

        extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
        extensionMotor.configPeakCurrentLimit(10)
        extensionMotor.configPeakCurrentDuration(100)

        on(Events.TELEOP_ENABLED){
            if (homed){
                armMachine.setState(ArmStates.MANUAL_CONTROL)
            }else{
                armMachine.setState(ArmStates.HOMING)
            }
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
    private fun chainReduction(value: Double): Double{
        return value * 16/72.0
    }


    override fun action() {
        position = rotationMotor.selectedSensorPosition.MagEncoderTicks.toUnit(Radians) as AngularDistanceMeasureRadians
        position = chainReduction(
            position.value
        ).Radians
        velocity = rotationMotor.selectedSensorVelocity.MagEncoderTicksPer100Ms.toUnit(RadiansPerSecond) as AngularVelocityMeasureRadiansPerSecond

        ArmKinematics.update(
            armLength,
            position
        )

        println("Position: $ArmKinematics")
    }


    val armMachine: StateMachine<ArmStates> = stateMachine {
        rejectAllIf(*ArmStates.values()){isInState(ArmStates.E_STOPPED)}

        state(ArmStates.E_STOPPED){
            entry {
                rotationMotor.set(ControlMode.PercentOutput, 0.0)
            }
            action {
                println("E Stopped")
            }
        }

        state(ArmStates.MANUAL_CONTROL){
            entry {
                rotationMotor.selectProfileSlot(0,0)
            }
            action {
                // Slowed down to not kill the arm for testing
                var input = Gamepad.readAxis { LEFT_Y }
                if (position.value <= ControlParameters.ArmParameters.MIN_POS && input < 0.0){
                    input = 0.0
                }
                if (position.value >= ControlParameters.ArmParameters.MAX_POS && input > 0.0){
                    input = 0.0
                }

                //println("Manual control $input")

                if (withinTolerance(0.0, input, 0.05)){
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
                println("Velocity: ${rotationMotor.selectedSensorVelocity.MagEncoderTicksPer100Ms.toUnit(RevolutionsPerSecond).value} encoder ticks / 100ms")
            }
        }

        state(ArmStates.HOLDING){
            entry {
                rotationMotor.selectProfileSlot(0,0)
                rotationMotor.set(ControlMode.MotionMagic, position.value)
            }
            action {
                println("Target Pos: ${position.value}")
            }
        }

        state(ArmStates.HOMING){
            var velocityCounter = 0
            entry {
                velocityCounter = 0
                rotationMotor.configMotionAcceleration(ArmParameters.MAX_ACCELERATION)
                rotationMotor.configMotionCruiseVelocity(ArmParameters.MAX_VELOCITY)
                rotationMotor.selectProfileSlot(1, 0)
                rotationMotor.set(ControlMode.Velocity, 0.5.RevolutionsPerSecond.toUnit(MagEncoderTicksPer100Ms).value)
            }
            action{
                println("Homing action")

                var velocity = rotationMotor.selectedSensorVelocity.MagEncoderTicksPer100Ms
                var current = rotationMotor.outputCurrent

                println("Velocity revs/s: ${velocity.toUnit(RevolutionsPerSecond).value}")
                println("Current : ${rotationMotor.outputCurrent}")

                if(abs(current) >= ArmParameters.HOMING_CURRENT){
                    velocityCounter++
                }else{
                    velocityCounter = 0
                }

                if(velocityCounter > 8){
                    rotationMotor.set(ControlMode.PercentOutput, 0.0)
                    homed = true
                    // Homed position for prototype: 215 deg, 3.75 rad
                    // Homed position is the Rear of the robot. - Motor power
                    println("Homed position: ${rotationMotor.selectedSensorPosition} ticks")
                    rotationMotor.selectedSensorPosition = 16.875.Radians.toUnit(MagEncoderTicks).value.toInt()
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
            action {
                println("E Stopped")
            }
        }
    }
}