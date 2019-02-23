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
import org.snakeskin.measure.Milliseconds
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.snakeskin.subsystem.SubsystemCheckContext
import org.snakeskin.utility.Ticker
import org.team401.robot2019.config.ControlParameters
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

    private val rotation = CTRESmartGearbox(rotationTalon)
    private val leftIntake = CTRESmartGearbox(leftIntakeTalon)
    private val rightIntake = CTRESmartGearbox(rightIntakeTalon)

    private val cargoSensor = DigitalInput(0)
    private val leftHatchSensor = DigitalInput(1)

    private val hatchPanelSolenoid = Solenoid(HardwareMap.Wrist.clawSolenoidID)
    private val cargoClampSolenoid = Solenoid(HardwareMap.Wrist.cargoClawSolenoidID)

    enum class WristStates {
        EStopped,
        CollectFf,
        GoTo90,
        GoTo0,
        GoTo180,
        Holding,
        CoordinatedControl
    }
    enum class ScoringStates {
        EStopped,
        IntakeCargo,
        EjectCargo,
        IntakeHatchPanel,
        ReleaseHatchPanel,
        Idle,
        CargoClamped,
        CargoReleased,
        HatchClamped,
        HatchReleased
    }

    private fun move(setpoint: AngularDistanceMeasureDegrees) {
        if (setpoint > 210.0.Degrees || setpoint < (-185.0).Degrees) {
            println("ILLEGAL WRIST COMMAND $setpoint")
            wristMachine.setState(WristStates.EStopped)
            return
        }
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

        state (WristStates.CoordinatedControl) {
            rtAction {
                val output = SuperstructureController.output
                val angle = output.wristTheta.toDegrees()

                move(angle)
            }
        }
    }

    val scoringMachine: StateMachine<ScoringStates> = stateMachine {
        rejectAllIf(*ScoringStates.values()){isInState(ScoringStates.EStopped)}

        state(ScoringStates.EStopped){
            entry{
                leftIntake.stop()
                rightIntake.stop()
                cargoClampSolenoid.set(true)
                hatchPanelSolenoid.set(true)
            }
        }
        state(ScoringStates.IntakeCargo){
            val sensingTimeout = Ticker(
                { systemSeesCargo()},
                ControlParameters.WristParameters.hasCargoTime,
                20.0.Milliseconds.toSeconds()
            )
            entry {
                sensingTimeout.reset()
                cargoClampSolenoid.set(false)
                leftIntake.set(ControlMode.PercentOutput, -0.8)
                rightIntake.set(ControlMode.PercentOutput, -0.8)
            }
            action{
                sensingTimeout.check {
                    systemSeesCargo()
                    setState(ScoringStates.Idle)
                }
            }
        }
        state(ScoringStates.EjectCargo){
            entry {
                leftIntake.set(ControlMode.PercentOutput, 1.0)
                rightIntake.set(ControlMode.PercentOutput, 1.0)
                setState(ScoringStates.Idle)
            }
        }
        state(ScoringStates.IntakeHatchPanel){
            entry {
                hatchPanelSolenoid.set(false)
            }
            action{
                if(systemSeesHatch()){
                    hatchPanelSolenoid.set(true)
                    setState(ScoringStates.Idle)
                }
            }
        }
        state(ScoringStates.ReleaseHatchPanel){
            entry {
                hatchPanelSolenoid.set(false)
                setState(ScoringStates.Idle)
            }
        }
        state(ScoringStates.Idle){
            entry {
                hatchPanelSolenoid.set(true)
                cargoClampSolenoid.set(true)
                leftIntake.set(ControlMode.PercentOutput, 0.0)
                rightIntake.set(ControlMode.PercentOutput, 0.0)
            }
        }

        state(ScoringStates.CargoClamped){
            entry {
                cargoClampSolenoid.set(true)
            }
        }
        state(ScoringStates.CargoReleased){
            entry {
                cargoClampSolenoid.set(false)
            }
        }
        state(ScoringStates.HatchClamped){
            entry {
                hatchPanelSolenoid.set(true)
            }
        }
        state(ScoringStates.HatchReleased){
            entry {
                hatchPanelSolenoid.set(false)
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
        //println("pwp: ${rotation.master.sensorCollection.pulseWidthPosition}\t pos: ${rotation.master.getSelectedSensorPosition(0)}  act: ${rotation.getPosition().toDegrees()}" )
        //println(rotation.getPosition().toDegrees())
    }

    override fun setup() {
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
            wristMachine.setState(WristStates.Holding)
        }
    }
}