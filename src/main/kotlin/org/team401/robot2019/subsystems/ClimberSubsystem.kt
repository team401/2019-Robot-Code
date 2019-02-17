package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.BaseMotorController
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DriverStation
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.Milliseconds
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.utility.Ticker
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.config.HardwareMap
import kotlin.math.roundToInt

/**
 * @author Cameron Earle
 * @version 2/15/2019
 *
 */
object ClimberSubsystem: Subsystem() {
    private val backTalon = TalonSRX(HardwareMap.Climber.backTalonId)
    private val frontTalon = TalonSRX(HardwareMap.Climber.frontTalonId)

    private val back = CTRESmartGearbox(backTalon)
    private val front = CTRESmartGearbox(frontTalon)

    enum class ClimberStates {
        Homing, //Home both sets of legs,  by driving them up at a fixed voltage until they stop moving
        Stowed, //Move both legs to their stowed position
        DownL2, //Moves both legs down to the position required to fall on to level 2
        DownL3, //Moves both legs down to the position required to fall on to level 3
        FallL2, //Retracts the front legs fully to "fall" onto level 2
        FallL3, //Retracts the front legs  fully to "fall" onto level 3
    }

    enum class ClimberFaults {
        MotorControllerReset,
        HomeLost
    }

    private fun upwardsMoveBack(setpointBack: LinearDistanceMeasureInches) {
        back.setPIDF(ControlParameters.ClimberParameters.BackUpPIDF)

        val nativeSetpointBack = setpointBack
            .toAngularDistance(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicks().value

        back.set(ControlMode.MotionMagic, nativeSetpointBack)
    }

    private fun upwardsMoveFront(setpointFront: LinearDistanceMeasureInches) {
        front.setPIDF(ControlParameters.ClimberParameters.FrontUpPIDF)

        val nativeSetpointFront = setpointFront
            .toAngularDistance(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicks().value

        front.set(ControlMode.MotionMagic, nativeSetpointFront)
    }

    private fun downwardsMoveBack(setpointBack: LinearDistanceMeasureInches) {
        back.setPIDF(ControlParameters.ClimberParameters.BackDownPIDF)

        val nativeSetpointBack = setpointBack
            .toAngularDistance(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicks().value

        back.set(ControlMode.MotionMagic, nativeSetpointBack)
    }

    private fun downwardsMoveFront(setpointFront: LinearDistanceMeasureInches) {
        front.setPIDF(ControlParameters.ClimberParameters.FrontDownPIDF)

        val nativeSetpointFront = setpointFront
            .toAngularDistance(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicks().value

        front.set(ControlMode.MotionMagic, nativeSetpointFront)
    }

    /**
     * Flag representing whether both legs are homed.  If one set is homed and one is not, they both need
     * to be rehomed
     */
    var homed by LockingDelegate(false)

    private val climberMachine: StateMachine<ClimberStates> = stateMachine {
        state (ClimberStates.Homing) {
            val ticker = Ticker(
                {back.getVelocity() == 0.0.RadiansPerSecond && front.getVelocity() == 0.0.RadiansPerSecond},
                ControlParameters.ClimberParameters.homingTime,
                20.0.Milliseconds.toSeconds()
            )

            entry {
                ticker.reset()
                back.set(ControlParameters.ClimberParameters.homingPower)
                front.set(ControlParameters.ClimberParameters.homingPower)
                println("Homing arm")
            }

            action {
                ticker.check {
                    //If we've stopped moving, and the time has elapsed, we're done
                    homed = true
                    setState(ClimberStates.Stowed)
                }
            }

            exit {
                back.stop()
                front.stop()
                println("Arm homed")
            }
        }

        state (ClimberStates.Stowed) {
            entry {
                upwardsMoveFront(ControlParameters.ClimberPositions.stowed)
                upwardsMoveBack(ControlParameters.ClimberPositions.stowed)
            }
        }
        
        state (ClimberStates.DownL2) {
            entry {
                downwardsMoveFront(ControlParameters.ClimberPositions.l2Climb)
                downwardsMoveBack(ControlParameters.ClimberPositions.l2Climb)
            }
        }
        
        state (ClimberStates.DownL3) {
            entry {
                downwardsMoveFront(ControlParameters.ClimberPositions.l3Climb)
                downwardsMoveBack(ControlParameters.ClimberPositions.l3Climb)
            }
        }
        
        state (ClimberStates.FallL2) {
            rejectIf {
                !isInState(ClimberStates.DownL2) //Don't allow us to fall unless we're already in the right up state
            }

            entry {
                upwardsMoveFront(ControlParameters.ClimberPositions.stowed)
                downwardsMoveBack(ControlParameters.ClimberPositions.l2Climb)
            }
        }

        state (ClimberStates.FallL3) {
            rejectIf {
                !isInState(ClimberStates.DownL3) //Don't allow us to fall unless we're already in the right up state
            }

            entry {
                upwardsMoveFront(ControlParameters.ClimberPositions.stowed)
                downwardsMoveBack(ControlParameters.ClimberPositions.l3Climb)
            }
        }
    }

    override fun action() {
        //Detect faults
        if (front.hasMasterResetOccurred() || back.hasMasterResetOccurred()) {
            fault(ClimberFaults.HomeLost)
            fault(ClimberFaults.MotorControllerReset)
        }

        if (front.hasAnyResetOccurred() || back.hasAnyResetOccurred()) {
            fault(ClimberFaults.MotorControllerReset)
        }

        //Respond to faults
        if (isFaulted(ClimberFaults.MotorControllerReset)) {
            DriverStation.reportWarning("[Fault] A climber motor controller has reset!", false)
            configureClimberMotorControllers()
            clearFault(ClimberFaults.MotorControllerReset)
            println("[Fault Cleared] Climber motor controllers reconfigured")
        }

        if (isFaulted(ClimberFaults.HomeLost)) {
            DriverStation.reportWarning("[Fault] Climber home lost!", false)
            homed = false
            //If we're enabled, home now.  Otherwise, the flag above will cause us to home on enable
            if (DriverStation.getInstance().isEnabled) {
                climberMachine.setState(ClimberStates.Homing)
            }
            clearFault(ClimberFaults.HomeLost)
            println("[Fault Cleared] Climber is homing")
        }
    }

    private fun configureClimberMotorControllers() {
        front.hasAnyResetOccurred()
        back.hasAnyResetOccurred()

        back.inverted = false
        front.inverted = false

        back.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        front.setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)

        back.setCurrentLimit(30.0, 0.0, 0.0.Seconds) //TODO put these in control parameters
        front.setCurrentLimit(30.0, 0.0, 0.0.Seconds) //TODO actually calculate these

        back.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
        front.setFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)

        //Configure motion velocities and accelerations
        val nativeVelocityBack = ControlParameters.ClimberParameters.climberVelocity
            .toAngularVelocity(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()
        val nativeVelocityFront = ControlParameters.ClimberParameters.climberVelocity
            .toAngularVelocity(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()

        val nativeAccelBack = ControlParameters.ClimberParameters.climberAcceleration
            .toAngularVelocity(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND
        val nativeAccelFront = ControlParameters.ClimberParameters.climberAcceleration
            .toAngularVelocity(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND
        
        back.master.configMotionCruiseVelocity(nativeVelocityBack)
        back.master.configMotionAcceleration(nativeAccelBack)
        front.master.configMotionCruiseVelocity(nativeVelocityFront)
        front.master.configMotionAcceleration(nativeAccelFront)
    }

    override fun setup() {
        configureClimberMotorControllers() //Initially configure controllers

        on (Events.ENABLED) {
            if (!homed) {
                //Home the climber
                climberMachine.setState(ClimberStates.Homing)
            } else {
                //Stow the climber
                climberMachine.setState(ClimberStates.Stowed)
            }
        }
    }
}