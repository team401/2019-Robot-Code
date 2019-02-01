package org.team401.robot2019.subsystems.arm

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.snakeskin.dsl.on
import org.snakeskin.dsl.rtAction
import org.snakeskin.dsl.stateMachine
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.state.StateMachine
import org.snakeskin.subsystem.Subsystem
import org.snakeskin.units.*
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.units.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.armsim.ArmKinematics
import org.team401.armsim.Point2d
import org.team401.armsim.PointPolar
import org.team401.robot2019.Gamepad
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.ControlParameters.ArmParameters
import org.team401.robot2019.config.Geometry
import kotlin.math.abs

object Arm: Subsystem() {
    private val rotationMotor = TalonSRX(20)
    private val extensionMotor = TalonSRX(21)
    private val wristMotor = TalonSRX(22)

    private var armAngle = rotationMotor.selectedSensorPosition.MagEncoderTicks
    private var armVelocity = rotationMotor.selectedSensorVelocity.MagEncoderTicksPer100Ms
    private var armLength = extensionMotor.selectedSensorPosition.MagEncoderTicks // linear

    private var wristAngle = wristMotor.selectedSensorPosition.MagEncoderTicks
    private var activeTool = Tool.CARGO_INTAKE // TODO Adjust how this works..
    private var hasGamePiece = false

    private var armPosition = ArmKinematics.forward(PointPolar(armLength.toLinearDistance(Geometry.ArmGeometry.armToInches) as LinearDistanceMeasureInches,
        armAngle.toUnit(Radians) as AngularDistanceMeasureRadians))

    private lateinit var coordinatedControlPoint: ArmSystemMotionPoint

    private var homed by LockingDelegate(false)

    override fun setup() {
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0)
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 0)
        rotationMotor.configPeakCurrentLimit(10)
        rotationMotor.configPeakCurrentDuration(100)
        rotationMotor.configClosedLoopPeakOutput(0, 0.25)
        rotationMotor.configClosedLoopPeakOutput(1, 0.25)

        // TODO Remember - Chain reduction is 16:72

        extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute)
        extensionMotor.configPeakCurrentLimit(10)
        extensionMotor.configPeakCurrentDuration(100)

        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute)
        wristMotor.configPeakCurrentLimit(10)
        wristMotor.configPeakCurrentDuration(100)

        // TODO SET UP MOTION MAGIC

        on(Events.TELEOP_ENABLED){
            if (homed){
                armMachine.setState(ArmStates.MANUAL_CONTROL)
            }else{
                armMachine.setState(ArmStates.HOMING)
            }
        }

    }
    enum class ArmStates{
        E_STOPPED, HOMING, MANUAL_CONTROL, COORDINATED_CONTROL, HOLDING, SWITCH_TOOL
    }

    private fun withinTolerance(target: Double, pos: Double, tolerance: Double): Boolean{
        return abs(target - pos) <= abs(tolerance)
    }
    private fun chainReduction(value: Double): Double{
        return value * 16/72.0
    }

    fun setTargetPosition(target: Point2d){
        ArmSubsystemController.commandMove(target)
    }

    override fun action() {
        armAngle = rotationMotor.selectedSensorPosition.MagEncoderTicks
        // TODO Update this to real robot configuration
        armAngle = chainReduction(armAngle.value).MagEncoderTicks
        armVelocity = rotationMotor.selectedSensorVelocity.MagEncoderTicksPer100Ms
        armPosition = ArmKinematics.forward(PointPolar(armLength.toLinearDistance(Geometry.ArmGeometry.armToInches) as LinearDistanceMeasureInches, armAngle.toUnit(Radians) as AngularDistanceMeasureRadians))

        wristAngle = wristMotor.selectedSensorPosition.MagEncoderTicks
        val armState = ArmState(armLength.toLinearDistance(Geometry.ArmGeometry.armToInches) as LinearDistanceMeasureInches, armAngle.toUnit(Radians) as AngularDistanceMeasureRadians, armVelocity.toUnit(
            RadiansPerSecond) as AngularVelocityMeasureRadiansPerSecond) // Double check arm length
        val wristState = WristState(wristAngle.toUnit(Radians) as AngularDistanceMeasureRadians, activeTool, hasGamePiece)

        coordinatedControlPoint = ArmSubsystemController.update(armState, wristState)
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

        state(ArmStates.MANUAL_CONTROL){// TODO REDO
            entry {
                rotationMotor.selectProfileSlot(0,0)
            }
            action {
                // Slowed down to not kill the arm for testing
                var input = Gamepad.readAxis { LEFT_Y }
                if (armAngle.value <= ControlParameters.ArmParameters.MIN_POS && input < 0.0){
                    input = 0.0
                }
                if (armAngle.value >= ControlParameters.ArmParameters.MAX_POS && input > 0.0){
                    input = 0.0
                }

                //println("Manual control $input")

                if (withinTolerance(0.0, input, 0.05)){
                    rotationMotor.set(ControlMode.MotionMagic, armAngle.value)
                }else {
                    rotationMotor.set(ControlMode.PercentOutput, 0.25 * input)
                }
            }
        }

        state(ArmStates.COORDINATED_CONTROL){
            rtAction {
                rotationMotor.config_kF(0, coordinatedControlPoint.rotationFeedForward)
                rotationMotor.set(ControlMode.Position, coordinatedControlPoint.targetPosition.value)

                extensionMotor.set(ControlMode.MotionMagic, coordinatedControlPoint.targetPosition.value)

                wristMotor.set(ControlMode.MotionMagic, coordinatedControlPoint.targetWristPosition.value)

                if(ArmSubsystemController.isDone() && armAngle == coordinatedControlPoint.targetPosition &&
                        armLength == coordinatedControlPoint.targetExtension && wristAngle == coordinatedControlPoint.targetWristPosition){// TODO add tolerances

                }
            }
        }

        state(ArmStates.HOLDING){
            entry {
                rotationMotor.selectProfileSlot(0,0) // TODO adjust these slots
                rotationMotor.set(ControlMode.MotionMagic, armAngle.value)

                extensionMotor.selectProfileSlot(0,0)
                extensionMotor.set(ControlMode.MotionMagic, armLength.value)

                wristMotor.selectProfileSlot(0, 0)
                wristMotor.set(ControlMode.MotionMagic, wristAngle.value)
            }
            action {
                println("Target Pos: ${armAngle.value}")
            }
        }

        state(ArmStates.SWITCH_TOOL){
            lateinit var newWristState: WristState
            entry {
                rotationMotor.selectProfileSlot(0,0)
                rotationMotor.set(ControlMode.MotionMagic, armAngle.value)

                extensionMotor.selectProfileSlot(0,0)
                extensionMotor.set(ControlMode.MotionMagic, armLength.value)

                if (activeTool == Tool.CARGO_INTAKE){ // Should probably be a boolean...
                    newWristState = ArmSubsystemController.switchTool(Tool.HATCH_INTAKE)
                }else{
                    newWristState = ArmSubsystemController.switchTool(Tool.CARGO_INTAKE)
                }
                wristMotor.set(ControlMode.MotionMagic, newWristState.wristPosition.toUnit(MagEncoderTicks).value)
            }
            action {
                if (wristAngle == newWristState.wristPosition.toUnit(MagEncoderTicks) as AngularDistanceMeasureCTREMagEncoder){
                    setState(ArmStates.HOLDING)
                }
            }
        }

        state(ArmStates.HOMING){
            var velocityCounter = 0
            entry {
                velocityCounter = 0
                rotationMotor.configMotionAcceleration(ArmParameters.MAX_ACCELERATION.value.toInt())
                rotationMotor.configMotionCruiseVelocity(ArmParameters.MAX_VELOCITY.value.toInt())
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
                    // Homed armAngle for prototype: 215 deg, 3.75 rad
                    // Homed armAngle is the Rear of the robot. - Motor power
                    println("Homed armAngle: ${rotationMotor.selectedSensorPosition} ticks")
                    rotationMotor.selectedSensorPosition = 16.875.Radians.toUnit(MagEncoderTicks).value.toInt()
                    setState(ArmStates.MANUAL_CONTROL)
                }
            }
        }
    }
}