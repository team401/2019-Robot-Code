package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.on
import org.snakeskin.dsl.rtAction
import org.snakeskin.dsl.stateMachine
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.MagEncoderTicks
import org.snakeskin.measure.MagEncoderTicksPerHundredMilliseconds
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RevolutionsPerSecond
import org.snakeskin.state.StateMachine
import org.snakeskin.subsystem.Subsystem
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.Gamepad
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.ControlParameters.ArmParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureControlOutput
import org.team401.robot2019.control.superstructure.geometry.*
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import kotlin.math.abs

object ArmSubsystem: Subsystem() {
    private val rotationMotor = TalonSRX(20)
    private val extensionMotor = TalonSRX(21)

    private val extension = CTRESmartGearbox(extensionMotor)
    private val rotation = CTRESmartGearbox(rotationMotor)

    private var armAngle = 0.0.MagEncoderTicks
    private var armVelocity = 0.0.MagEncoderTicksPerHundredMilliseconds
    private var armLength = 0.0.MagEncoderTicks

    //TODO move a lot of this out of the subsystem and into the planner

    private var wristAngle = 0.0.MagEncoderTicks
    private var activeTool = WristMotionPlanner.Tool.CargoTool// TODO Adjust how this works..
    private var hasGamePiece = false

    private lateinit var armPosition: Point2d

    private lateinit var coordinatedControlPoint: SuperstructureControlOutput

    private var homed by LockingDelegate(false)

    /**
     * Gets the current state of the arm as measured by sensors
     */
    fun getCurrentArmState(): ArmState {
        val extensionLength = extension.getPosition()
            .toLinearDistance(Geometry.ArmGeometry.extensionAngularToLinearRadius)
            .toInches()
        val radius = extensionLength + Geometry.ArmGeometry.armBaseLength

        val armAngle = rotation.getPosition()
        val armVelocity = rotation.getVelocity()

        return ArmState(radius, armAngle, armVelocity)
    }

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

        // TODO SET UP MOTION MAGIC

        extensionMotor.configMotionCruiseVelocity(ControlParameters.ArmParameters.EXTENSION_MAX_VELOCITY)
        extensionMotor.configMotionAcceleration(ControlParameters.ArmParameters.EXTENSION_MAX_ACCELERATION)

        on(Events.TELEOP_ENABLED){
            armMachine.setState(ArmStates.HOLDING)
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

    override fun action() {
        armAngle = rotationMotor.selectedSensorPosition.toDouble().MagEncoderTicks
        // TODO Update this to real robot configuration
        armAngle = chainReduction(armAngle.value)
            .MagEncoderTicks
        armLength = extensionMotor.selectedSensorPosition.toDouble().MagEncoderTicks
        armVelocity = rotationMotor.selectedSensorVelocity.toDouble().MagEncoderTicksPerHundredMilliseconds
        armPosition = ArmKinematics.forward(PointPolar((armLength.toLinearDistance(Geometry.ArmGeometry.extensionAngularToLinearRadius) + Geometry.ArmGeometry.minSafeWristRotationHeight), armAngle.toRadians()))


        val armState = ArmState(
            armLength.toLinearDistance(Geometry.ArmGeometry.extensionAngularToLinearRadius),
            armAngle.toRadians(),
            armVelocity.toRadiansPerSecond()
        ) // Double check superstructure length

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
                // Slowed down to not kill the superstructure for testing
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
                /*
                rotationMotor.config_kF(0, coordinatedControlPoint.rotationFeedForward)
                rotationMotor.set(ControlMode.Position, coordinatedControlPoint.targetPosition.value)

                extensionMotor.set(ControlMode.MotionMagic, coordinatedControlPoint.targetPosition.value)

                wristMotor.set(ControlMode.MotionMagic, coordinatedControlPoint.targetWristPosition.value)

                if(SuperstructureMotionPlanner.isDone() && armAngle == coordinatedControlPoint.targetPosition &&
                        armLength == coordinatedControlPoint.extension && wristAngle == coordinatedControlPoint.targetWristPosition){// TODO add tolerances

                }
                */

                //TODO update this to use the arb ff properly
            }
        }

        state(ArmStates.HOLDING){
            entry {
                rotationMotor.selectProfileSlot(0,0) // TODO adjust these slots
                rotationMotor.set(ControlMode.MotionMagic, armAngle.value)

                extensionMotor.selectProfileSlot(0,0)
                extensionMotor.set(ControlMode.MotionMagic, armLength.value)
            }
        }
    }
}