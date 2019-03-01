package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.SensorCollection
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Solenoid
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.DegreesPerSecond
import org.snakeskin.measure.Milliseconds
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.snakeskin.subsystem.SubsystemCheckContext
import org.snakeskin.utility.Ticker
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.WristState
import kotlin.math.roundToInt

/**
 * @author Cameron Earle
 * @version 2/10/2019
 *
 */
object WristSubsystem: Subsystem() {
    private val rotationTalon = TalonSRX(HardwareMap.Arm.wristTalonId)
    val leftIntakeTalon = TalonSRX(HardwareMap.Arm.leftIntakeWheelTalonId)
    val rightIntakeTalon = TalonSRX(HardwareMap.Arm.rightIntakeWheelTalonId)

    val cargoIntake = CTRESmartGearbox(leftIntakeTalon, rightIntakeTalon)

    private val rotation = CTRESmartGearbox(rotationTalon)
    private val leftIntake = CTRESmartGearbox(leftIntakeTalon)
    private val rightIntake = CTRESmartGearbox(rightIntakeTalon)

    private val cargoSensor = DigitalInput(0)
    private val leftHatchSensor = DigitalInput(1)

    private val hatchPanelSolenoid = Solenoid(HardwareMap.Wrist.clawSolenoidID)
    private val cargoGrabberSolenoid = Solenoid(HardwareMap.Wrist.cargoClawSolenoidID)

    enum class WristStates {
        EStopped,
        CollectFf,
        GoTo0,
        Holding,
        CoordinatedControl
    }

    enum class CargoGrabberStates {
        Clamped,
        Unclamped
    }

    enum class CargoWheelsStates {
        Intake,
        Idle,
        Scoring
    }

    /*
    enum class HatchClawStates {
        Clamped, //Clamped around a gamepiece
        Unclamped
    }
    */

    private fun move(setpoint: AngularDistanceMeasureDegrees) {
        if (setpoint > 210.0.Degrees || setpoint < (-185.0).Degrees) {
            println("ILLEGAL WRIST COMMAND $setpoint")
            wristMachine.setState(WristStates.EStopped)
            return
        }
        rotation.set(ControlMode.MotionMagic, setpoint.toMagEncoderTicks().value)
    }

    val wristMachine: StateMachine<WristStates> = stateMachine {
        rejectAllIf(*WristStates.values()){isInState(WristStates.EStopped)}

        state (WristStates.EStopped) {
            action {
                rotation.set(0.0)
            }
        }

        state (WristStates.CollectFf) {
            var lastVel = 0
            entry {
                rotation.set(0.5)
            }

            action {
                lastVel = rotation.master.getSelectedSensorVelocity(0)
            }

            exit {
                rotation.stop()
                println("Wrist FF: ${0.5 * 1023.0 / lastVel}")
            }
        }

        state (WristStates.Holding) {
            entry {
                val position = rotation.getPosition().toDegrees()
                move(position)
            }
        }

        state (WristStates.GoTo0) {
            entry {
                val position = rotation.getPosition().toDegrees()
                move(position)
            }

            action {
                if (ArmSubsystem.getCurrentArmState().armRadius >= Geometry.ArmGeometry.minSafeArmLength) {
                    //println("MOVING TO 0")
                    move(0.0.Degrees)
                }
            }
        }

        state (WristStates.CoordinatedControl) {
            rtAction {
                val output = SuperstructureController.output
                val angle = output.wristTheta.toDegrees()

                move(angle)
            }
        }

        disabled {
            action {
                rotation.set(0.0)
            }
        }
    }

    val cargoGrabberMachine: StateMachine<CargoGrabberStates> = commandMachine(
        stateMap(
            CargoGrabberStates.Clamped to false,
            CargoGrabberStates.Unclamped to true
        )
    ) {
        cargoGrabberSolenoid.set(value)
    }

    val cargoWheelsMachine: StateMachine<CargoWheelsStates> = commandMachine(
        stateMap(
            CargoWheelsStates.Idle to 0.0,
            CargoWheelsStates.Intake to ControlParameters.WristParameters.intakePower,
            CargoWheelsStates.Scoring to ControlParameters.WristParameters.scoringPower
        )
    ) {
        cargoIntake.set(value)
    }

    /**
     * Returns whether or not the sensor system sees a cargo present in the intake.
     * The sensor should be located in the front of the intake such that it can be used
     * to both automatically close the intake as well as detect continuous presence.
     */
    private fun systemSeesCargo(): Boolean {
        //return cargoSensor.get() //TODO check polarity of banner sensor
        return false
    }

    /**
     * Returns whether or not the sensor system sees a hatch present in the claw.
     * The sensor(s) should be located such that they reliably indicate the presence
     * of a hatch regardless of orientation or position in the claw
     */
    private fun systemSeesHatch(): Boolean {
        return false //TODO add sensor
    }

    /**
     * Gets the current state of the wrist as measured by sensors
     */
    fun getCurrentWristState(): WristState {
        val wristAngle = rotation.getPosition()

        return WristState(wristAngle, systemSeesCargo(), systemSeesHatch())
    }

    override fun action() {
        //println(cargoSensor.get())
        //println("pwp: ${rotation.master.sensorCollection.pulseWidthPosition}\t pos: ${rotation.master.getSelectedSensorPosition(0)}  act: ${rotation.getPosition().toDegrees()}" )
        //println(rotation.getPosition().toDegrees())
    }

    override fun setup() {
        leftIntakeTalon.inverted = true
        rightIntakeTalon.inverted = false
        rotation.inverted = false
        rotation.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        rotation.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
        rotation.master.selectedSensorPosition = Math.abs(rotation.master.sensorCollection.pulseWidthPosition % 4096.0).roundToInt() - 2396 + (2048)
        rotation.setPIDF(ControlParameters.WristParameters.WristRotationPIDF)
        rotation.setCurrentLimit(30.0, 0.0, 0.0.Seconds)

        rotation.master.configMotionCruiseVelocity(
            ControlParameters.WristParameters.cruiseVelocity
                .toMagEncoderTicksPerHundredMilliseconds()
                .value.roundToInt()
        )

        rotation.master.configMotionAcceleration(
            ControlParameters.WristParameters.acceleration
                .toMagEncoderTicksPerHundredMillisecondsPerSecond()
                .value.roundToInt()
        )

        on (Events.ENABLED) {
            wristMachine.setState(WristStates.GoTo0)
        }
    }
}