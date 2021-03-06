package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Solenoid
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.logic.History
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureDegrees
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.util.LEDManager
import kotlin.math.roundToInt

/**
 * @author Cameron Earle
 * @version 2/10/2019
 *
 */
object WristSubsystem: Subsystem(100L) {
    private val rotationTalon = TalonSRX(HardwareMap.CAN.wristPivotTalonId)
    val leftIntakeTalon = TalonSRX(HardwareMap.CAN.wristLeftIntakeWheelTalonId)
    val rightIntakeTalon = TalonSRX(HardwareMap.CAN.wristRightIntakeWheelTalonId)

    val cargoIntake = CTRESmartGearbox(leftIntakeTalon, rightIntakeTalon)

    private val rotation = CTRESmartGearbox(rotationTalon)
    private val leftIntake = CTRESmartGearbox(leftIntakeTalon)
    private val rightIntake = CTRESmartGearbox(rightIntakeTalon)

    private val cargoHistory = History<Boolean>()

    private val hatchClawSolenoid = Solenoid(HardwareMap.Pneumatics.hatchClawSolenoidId)
    private val cargoGrabberSolenoid = Solenoid(HardwareMap.Pneumatics.cargoClawSolenoidId)

    enum class WristStates {
        EStopped,
        CollectFf,
        GoTo0,
        ForceTo0,
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
        Scoring,
        Holding
    }

    enum class HatchClawStates {
        Clamped, //Clamped around a gamepiece
        Unclamped
    }

    enum class WristFaults {
        WristEncoderFault,
        BannerSensorFault
    }

    private fun move(setpoint: AngularDistanceMeasureDegrees) {
        /*
        if (setpoint > 210.0.Degrees || setpoint < (-185.0).Degrees) {
            println("ILLEGAL WRIST COMMAND $setpoint")
            wristMachine.setState(WristStates.EStopped)
            return
        }
        */
        rotation.set(ControlMode.MotionMagic, setpoint.toMagEncoderTicks().value)
    }

    val wristMachine: StateMachine<WristStates> = stateMachine {
        //rejectAllIf(*WristStates.values()){isInState(WristStates.EStopped)}

        state (WristStates.EStopped) {
            entry {
                println("WRIST E STOP")
            }
            action {
                rotation.set(0.0)
            }
            exit {
                println("Wrist is operational")
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
                rotation.set(0.0)
            }

            action {
                if (ArmSubsystem.getCurrentArmState().armRadius >= Geometry.ArmGeometry.minSafeArmLength) {
                    move(0.0.Degrees)
                }
            }
        }

        state (WristStates.ForceTo0) {
            entry {
                move(0.0.Degrees)
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

    val cargoGrabberMachine: StateMachine<CargoGrabberStates> = stateMachine {
        state (CargoGrabberStates.Unclamped) {
            entry {
                cargoGrabberSolenoid.set(true)
            }
        }

        state (CargoGrabberStates.Clamped) {
            entry {
                cargoGrabberSolenoid.set(false)
            }

            action {
                if (SuperstructureController.output.wristTool == WristMotionPlanner.Tool.HatchPanelTool) {
                    setState(CargoGrabberStates.Unclamped) //Open the grabber if we're in the hatch tool
                }
            }
        }
    }

    val hatchClawMachine: StateMachine<HatchClawStates> = stateMachine {
        state (HatchClawStates.Unclamped) {
            entry {
                hatchClawSolenoid.set(true)
            }

            /*
            action {
                if (SuperstructureController.output.wristTool == WristMotionPlanner.Tool.CargoTool) {
                    setState(HatchClawStates.Unclamped) //Open (clamp) the claw if we're in the cargo tool
                }
            }
            */
        }

        state (HatchClawStates.Clamped) {
            entry {
                hatchClawSolenoid.set(false)
            }

            action {
                if (SuperstructureController.output.wristTool == WristMotionPlanner.Tool.CargoTool) {
                    setState(HatchClawStates.Unclamped) //Close (unclamp) the claw if we're in the cargo tool
                }
            }
        }
    }

    val cargoWheelsMachine: StateMachine<CargoWheelsStates> = stateMachine {
        state (CargoWheelsStates.Idle) {
            entry {
                leftIntake.set(0.0)
                rightIntake.set(0.0)
            }

            action {
                if (SuperstructureController.output.wristTool == WristMotionPlanner.Tool.CargoTool) {
                    setState(CargoWheelsStates.Holding)
                }
            }
        }

        state (CargoWheelsStates.Intake) {
            entry {
                leftIntake.set(ControlParameters.WristParameters.intakePower)
                rightIntake.set(ControlParameters.WristParameters.intakePower)
            }

            action {
                /*
                cargoHistory.update(cargoSensorNO.get())
                if (cargoHistory.current == true && cargoHistory.last == false) {
                    send(RobotEvents.CargoAcquired)
                    cargoGrabberMachine.setState(CargoGrabberStates.Clamped)
                    setState(WristSubsystem.CargoWheelsStates.Idle)
                }
                */
            }
        }

        state (CargoWheelsStates.Scoring) {
            entry {
                leftIntake.set(ControlParameters.WristParameters.scoringPower)
                rightIntake.set(ControlParameters.WristParameters.scoringPower)
            }
        }

        state (CargoWheelsStates.Holding) {
            entry {
                leftIntake.set(ControlParameters.WristParameters.holdingPower)
                rightIntake.set(ControlParameters.WristParameters.holdingPower)
            }

            action {
                if (SuperstructureController.output.wristTool != WristMotionPlanner.Tool.CargoTool) {
                    setState(CargoWheelsStates.Idle)
                }
            }
        }
    }

    /**
     * Returns whether or not the sensor system sees a cargo present in the intake.
     * The sensor should be located in the front of the intake such that it can be used
     * to both automatically close the intake as well as detect continuous presence.
     */
    private fun systemSeesCargo(): Boolean {
        //return cargoSensorNO.get()
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
        LEDManager.updateGamepieceStatus(systemSeesHatch(), systemSeesCargo()) //Update gamepiece status from sensors


        // Faults
        if (rotation.getSensorCollection().pulseWidthRiseToRiseUs == 0) {
            fault(WristFaults.WristEncoderFault)
            DriverStation.reportWarning("[Fault] Wrist Encoder has failed!", false)
        }


        // Driver Station Shutoff
        /*
        if (DriverStationDisplay.wristStopped.getBoolean(false)) {
            wristMachine.setState(WristStates.EStopped)
        }else if (wristMachine.isInState(WristStates.EStopped) && !DriverStationDisplay.wristStopped.getBoolean(false)) {
            wristMachine.setState(WristStates.Holding)
        }
        */
        //debug
        //
        //println("Raw: ${pot.value}\tDegrees: ${getPotAngleDegrees()}\tEnc: ${rotation.getPosition().toDegrees()}")
        //println("Wrist angle: ${rotation.getPosition().toDegrees()}")
    }

    override fun setup() {
        leftIntakeTalon.inverted = false
        rightIntakeTalon.inverted = true

        println("PWP: ${rotation.master.sensorCollection.pulseWidthPosition} set : ${rotation.master.selectedSensorPosition}")

        DrivetrainSubsystem.configureFeedbackTalonsForDrive(leftIntakeTalon, rightIntakeTalon)

        rotation.inverted = false
        rotation.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        rotation.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)

        /*
        if (Math.abs(rotation.master.selectedSensorPosition - rotation.master.sensorCollection.pulseWidthPosition) >= 10.0) {
            //We need to reset the wrist home
            rotation.master.selectedSensorPosition = Math.abs(rotation.master.sensorCollection.pulseWidthPosition % 4096.0).roundToInt() - 2396 + (2048)
        }
        */

        /*
        val samples = arrayListOf<Double>()

        for (i in 0 until 10) {
            samples.add(getPotAngleDegrees().value) //Take sample
            Thread.sleep(10) //Wait ~10 ms
        }

        rotation.master.selectedSensorPosition = samples.average().Degrees.toMagEncoderTicks().value.roundToInt()

*/
        //We're using this status frame that we don't plan on using to determine whether the talon has ever been
        //initialized by the application.  This is what is known in the industry as a HACK
        if (rotation.master.getStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000) >= 240) {
            //Sensor offset not set, configure it now
            rotation.master.selectedSensorPosition = (180.0).Degrees.toMagEncoderTicks().value.roundToInt()
            //We homed the sensor, blink the lights for a second to indicate this.
            LEDManager.signalTruss(LEDManager.TrussLedSignal.WristHomed)
            rotation.master.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000, 1000)
        }

        println("ROTATION FRAME 14: ${rotation.master.getStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000)}")

        rotation.setPIDF(ControlParameters.WristParameters.WristRotationPIDF)
        rotation.setCurrentLimit(30.0, 0.0, 0.0.Seconds)

        rotation.master.configMotionCruiseVelocity(
            ControlParameters.WristParameters.velocity
                .toMagEncoderTicksPerHundredMilliseconds()
                .value.roundToInt()
        )

        rotation.master.configMotionAcceleration(
            ControlParameters.WristParameters.acceleration
                .toMagEncoderTicksPerHundredMillisecondsPerSecond()
                .value.roundToInt()
        )

        on (Events.ENABLED) {
            cargoWheelsMachine.setState(CargoWheelsStates.Idle)
            cargoGrabberMachine.setState(CargoGrabberStates.Clamped)
            hatchClawMachine.setState(HatchClawStates.Clamped)

            /*
            val armState = ArmKinematics.forward(ArmSubsystem.getCurrentArmState())
            val currentTool = when {
                armState.x >= 0.0.Inches -> WristMotionPlanner.Tool.HatchPanelTool
                else -> WristMotionPlanner.Tool.CargoTool
            }
            SuperstructureMotionPlanner.activeTool = currentTool
            */

            wristMachine.setState(WristStates.Holding)
        }
    }
}