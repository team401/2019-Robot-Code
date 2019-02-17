package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.SensorCollection
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.HardwareMap
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

    private val rotation = CTRESmartGearbox(rotationTalon)
    private val leftIntake = CTRESmartGearbox(leftIntakeTalon)
    private val rightIntake = CTRESmartGearbox(rightIntakeTalon)

    private val cargoSensor = DigitalInput(0)
    private val leftHatchSensor = DigitalInput(1)

    enum class WristStates {
        CollectFf,
        GoTo90,
        GoTo0,
        GoTo180
    }

    private fun move(setpoint: AngularDistanceMeasureDegrees) {
        rotation.set(ControlMode.MotionMagic, setpoint.toMagEncoderTicks().value)
    }

    val wristMachine: StateMachine<WristStates> = commandMachine(
        stateMap(
            WristStates.GoTo90 to 90.0.Degrees,
            WristStates.GoTo180 to 180.0.Degrees,
            WristStates.GoTo0 to 0.0.Degrees
        ),
        { move(value)}
    ) {
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
        println(rotation.master.sensorCollection.pulseWidthPosition)
        //println(rotation.getPosition().toDegrees())
    }

    override fun setup() {
        rotation.inverted = false
        rotation.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        rotation.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute)
        rotation.master.selectedSensorPosition = Math.abs(rotation.master.sensorCollection.pulseWidthPosition % 4096.0).roundToInt() - 2305 + (2048)
        rotation.setPIDF(ControlParameters.WristParameters.WristRotationPIDF)
        rotation.setCurrentLimit(30.0, 0.0, 0.0.Seconds)
        //rotation.master.setSelectedSensorPosition()

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

        on (Events.TELEOP_ENABLED) {
            wristMachine.setState(WristStates.GoTo90)
        }
    }
}