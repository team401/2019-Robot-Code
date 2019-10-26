package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.*
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
import org.team401.robot2019.util.MathUtil
import kotlin.math.abs
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

    private val rotation = CTRESmartGearbox(rotationTalon)
    private val leftIntake = CTRESmartGearbox(leftIntakeTalon)
    private val rightIntake = CTRESmartGearbox(rightIntakeTalon)

    private val wristToolSolenoid = Solenoid(HardwareMap.Pneumatics.cargoClawSolenoidId)

    enum class WristStates {
        EStopped,
        CollectFf,
        GoTo0,
        ForceTo0,
        Holding,
        CoordinatedControl
    }

    enum class WristToolStates {
        Hatch,
        Cargo
    }

    enum class WristWheelsStates {
        Intake,
        Idle,
        Scoring,
        Holding
    }

    enum class WristFaults {
        WristEncoderFault,
        BannerSensorFault
    }

    private fun move(setpoint: AngularDistanceMeasureDegrees) {
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

    val toolMachine: StateMachine<WristToolStates> = stateMachine {
        state (WristToolStates.Hatch) {
            entry {
                wristToolSolenoid.set(false)
            }

            action {
                if (SuperstructureController.output.wristTool == WristMotionPlanner.Tool.CargoTool) {
                    setState(WristToolStates.Cargo)
                }
            }
        }

        state (WristToolStates.Cargo) {
            entry {
                wristToolSolenoid.set(true)
            }

            action {
                if (SuperstructureController.output.wristTool == WristMotionPlanner.Tool.HatchPanelTool) {
                    setState(WristToolStates.Hatch)
                }
            }
        }
    }

    val wheelsMachine: StateMachine<WristWheelsStates> = stateMachine {
        state (WristWheelsStates.Idle) {
            entry {
                leftIntake.set(0.0)
                rightIntake.set(0.0)
            }
        }

        state (WristWheelsStates.Intake) {
            action {
                when (SuperstructureController.output.wristTool) {
                    WristMotionPlanner.Tool.HatchPanelTool -> {
                        leftIntake.set(ControlParameters.WristParameters.hatchIntakePower)
                        rightIntake.set(ControlParameters.WristParameters.hatchIntakePower)
                    }
                    WristMotionPlanner.Tool.CargoTool -> {
                        leftIntake.set(ControlParameters.WristParameters.cargoIntakePower)
                        rightIntake.set(ControlParameters.WristParameters.cargoIntakePower)
                    }
                }
            }
        }

        state (WristWheelsStates.Scoring) {
            action {
                when (SuperstructureController.output.wristTool) {
                    WristMotionPlanner.Tool.HatchPanelTool -> {
                        leftIntake.set(ControlParameters.WristParameters.hatchScoringPower)
                        rightIntake.set(ControlParameters.WristParameters.hatchScoringPower)
                    }
                    WristMotionPlanner.Tool.CargoTool -> {
                        leftIntake.set(ControlParameters.WristParameters.cargoScoringPower)
                        rightIntake.set(ControlParameters.WristParameters.cargoScoringPower)
                    }
                }
            }
        }

        state (WristWheelsStates.Holding) {
            action {
                when (SuperstructureController.output.wristTool) {
                    WristMotionPlanner.Tool.HatchPanelTool -> {
                        leftIntake.set(ControlParameters.WristParameters.hatchHoldingPower)
                        rightIntake.set(ControlParameters.WristParameters.hatchHoldingPower)
                    }
                    WristMotionPlanner.Tool.CargoTool -> {
                        leftIntake.set(ControlParameters.WristParameters.cargoHoldingPower)
                        rightIntake.set(ControlParameters.WristParameters.cargoHoldingPower)
                    }
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

    private fun configWristHome(force: Boolean = false) {
        val ticks = rotationTalon.sensorCollection.pulseWidthPosition
        val offset = MathUtil.offsetEncoder12B(ticks, 3660, 3072)
        rotationTalon.selectedSensorPosition = offset
    }

    override fun action() {
        LEDManager.updateGamepieceStatus(systemSeesHatch(), systemSeesCargo()) //Update gamepiece status from sensors


        // Faults
        if (rotation.getSensorCollection().pulseWidthRiseToRiseUs == 0) {
            fault(WristFaults.WristEncoderFault)
            DriverStation.reportWarning("[Fault] Wrist Encoder has failed!", false)
        }

        //println(rotationTalon.selectedSensorPosition)
    }

    override fun setup() {
        leftIntakeTalon.inverted = false
        rightIntakeTalon.inverted = true

        rotation.inverted = false
        rotation.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        rotation.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 1000)

        configWristHome()

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
            wheelsMachine.setState(WristWheelsStates.Idle)
            toolMachine.setState(WristToolStates.Hatch)
            wristMachine.setState(WristStates.Holding)
            //Thread.sleep(5000)
            //wristMachine.setState(WristStates.CollectFf)
        }
    }
}