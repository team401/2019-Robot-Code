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
import kotlin.math.abs

object PrototypeArm: Subsystem() {


    // TODO EXTENSION AND ROTATION HAVE MERGED. REVIEW ALL CODE TO INCLUDE EXTENSION
    private val rotationMotor = TalonSRX(20)
    private val extensionMotor = TalonSRX(21)

    private var armPosition = rotationMotor.selectedSensorPosition.Radians
    private var armVelocity = rotationMotor.selectedSensorVelocity.RadiansPerSecond
    private var armLength = extensionMotor.selectedSensorPosition.MagEncoderTicks // TODO fix this value

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

    private fun withinTolerance(target: Double, pos: Double, tolerance: Double): Boolean{
        return abs(target - pos) <= abs(tolerance)
    }
    private fun chainReduction(value: Double): Double{
        return value * 16/72.0
    }


    override fun action() {
        armPosition = rotationMotor.selectedSensorPosition.MagEncoderTicks.toUnit(Radians) as AngularDistanceMeasureRadians
        armPosition = chainReduction(armPosition.value).Radians
        armVelocity = rotationMotor.selectedSensorVelocity.MagEncoderTicksPer100Ms.toUnit(RadiansPerSecond) as AngularVelocityMeasureRadiansPerSecond
    }


    val armMachine: StateMachine<ArmStates> = stateMachine {
        rejectAllIf(*ArmStates.values()){isInState(ArmStates.E_STOPPED)}

        state(ArmStates.E_STOPPED){
            entry {
                rotationMotor.set(ControlMode.PercentOutput, 0.0)
                extensionMotor.set(ControlMode.PercentOutput, 0.0)
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
                if (armPosition.value <= ControlParameters.ArmParameters.MIN_POS && input < 0.0){
                    input = 0.0
                }
                if (armPosition.value >= ControlParameters.ArmParameters.MAX_POS && input > 0.0){
                    input = 0.0
                }

                //println("Manual control $input")

                if (withinTolerance(0.0, input, 0.05)){
                    rotationMotor.set(ControlMode.MotionMagic, armPosition.value)
                }else {
                    rotationMotor.set(ControlMode.PercentOutput, 0.25 * input)
                }
            }
        }

        state(ArmStates.BETTER_CONTROL){
            action {

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
                rotationMotor.set(ControlMode.MotionMagic, armPosition.value)

                extensionMotor.selectProfileSlot(0,0)
                extensionMotor.set(ControlMode.MotionMagic, armLength.value)
            }
            action {
                println("Target Pos: ${armPosition.value}")
            }
        }

        state(ArmStates.HOMING){
            var velocityCounter = 0
            entry {
                velocityCounter = 0
                rotationMotor.configMotionAcceleration(ArmParameters.MAX_ACCELERATION.toInt())
                rotationMotor.configMotionCruiseVelocity(ArmParameters.MAX_VELOCITY.toInt())
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
                    // Homed armPosition for prototype: 215 deg, 3.75 rad
                    // Homed armPosition is the Rear of the robot. - Motor power
                    println("Homed armPosition: ${rotationMotor.selectedSensorPosition} ticks")
                    rotationMotor.selectedSensorPosition = 16.875.Radians.toUnit(MagEncoderTicks).value.toInt()
                    setState(ArmStates.MANUAL_CONTROL)
                }
            }
        }
    }
}