package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.Inches
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
        Disabled,
        TestDown,
        Homing, //Home both sets of legs,  by driving them up at a fixed voltage until they stop moving
        Stowed, //Move both legs to their stowed position
        DownL2, //Moves both legs down to the position required to fall on to level 2
        DownL3, //Moves both legs down to the position required to fall on to level 3
        FallL2, //Retracts the front legs fully to "fall" onto level 2
        FallL3, //Retracts the front legs  fully to "fall" onto level 3
        SlowFall, //Slowly retracts the legs. Used if we are aborting a climb
        LondonBridgeIsMaybeFallingDown // Stows both legs. To separate stowed from climbing
    }

    enum class ClimberFaults {
        MotorControllerReset,
        HomeLost
    }

    fun getFrontHeightInches(): LinearDistanceMeasureInches {
        return front.getPosition().toLinearDistance(Geometry.ClimberGeometry.frontPitchRadius)
    }

    fun getBackHeightInches(): LinearDistanceMeasureInches {
        return back.getPosition().toLinearDistance(Geometry.ClimberGeometry.backPitchRadius)
    }

    fun frontWithinTolerance(setpoint: LinearDistanceMeasureInches): Boolean {
        return Math.abs((setpoint - getFrontHeightInches()).value).Inches <= ControlParameters.ClimberParameters.climberTolerance
    }

    fun backWithinTolerance(setpoint: LinearDistanceMeasureInches): Boolean {
        return Math.abs((setpoint - getBackHeightInches()).value).Inches <= ControlParameters.ClimberParameters.climberTolerance
    }

    private fun upwardsMoveBack(setpointBack: LinearDistanceMeasureInches) {
        //Configure motion velocities and accelerations
        val nativeVelocityBack = ControlParameters.ClimberParameters.climberVelocityUp
            .toAngularVelocity(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()

        val nativeAccelBack = ControlParameters.ClimberParameters.climberAccelerationUp
            .toAngularVelocity(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND

        back.master.configMotionCruiseVelocity(nativeVelocityBack, 0)
        back.master.configMotionAcceleration(nativeAccelBack, 0)


        back.setPIDF(ControlParameters.ClimberParameters.BackUpPIDF, 0, 0)

        val nativeSetpointBack = setpointBack
            .toAngularDistance(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicks().value

        back.set(ControlMode.MotionMagic, nativeSetpointBack)
    }

    private fun upwardsMoveFront(setpointFront: LinearDistanceMeasureInches) {
        //Configure motion velocities and accelerations
        val nativeVelocityFront = ControlParameters.ClimberParameters.climberVelocityUp
            .toAngularVelocity(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()

        val nativeAccelFront = ControlParameters.ClimberParameters.climberAccelerationUp
            .toAngularVelocity(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND

        front.master.configMotionCruiseVelocity(nativeVelocityFront, 0)
        front.master.configMotionAcceleration(nativeAccelFront, 0)

        front.setPIDF(ControlParameters.ClimberParameters.FrontUpPIDF, 0, 0)

        val nativeSetpointFront = setpointFront
            .toAngularDistance(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicks().value

        front.set(ControlMode.MotionMagic, nativeSetpointFront)
    }

    private fun upwardsMoveBackSlow(setpointBack: LinearDistanceMeasureInches) {
        //Configure motion velocities and accelerations
        val nativeVelocityBack = ControlParameters.ClimberParameters.climberVelocityUpSlow
            .toAngularVelocity(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()

        val nativeAccelBack = ControlParameters.ClimberParameters.climberAccelerationUpSlow
            .toAngularVelocity(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND

        back.master.configMotionCruiseVelocity(nativeVelocityBack, 0)
        back.master.configMotionAcceleration(nativeAccelBack, 0)


        back.setPIDF(ControlParameters.ClimberParameters.BackUpPIDF, 0, 0)

        val nativeSetpointBack = setpointBack
            .toAngularDistance(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicks().value

        back.set(ControlMode.MotionMagic, nativeSetpointBack)
    }

    private fun upwardsMoveFrontSlow(setpointFront: LinearDistanceMeasureInches) {
        //Configure motion velocities and accelerations
        val nativeVelocityFront = ControlParameters.ClimberParameters.climberVelocityUpSlow
            .toAngularVelocity(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()

        val nativeAccelFront = ControlParameters.ClimberParameters.climberAccelerationUpSlow
            .toAngularVelocity(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND

        front.master.configMotionCruiseVelocity(nativeVelocityFront, 0)
        front.master.configMotionAcceleration(nativeAccelFront, 0)

        front.setPIDF(ControlParameters.ClimberParameters.FrontUpPIDF, 0, 0)

        val nativeSetpointFront = setpointFront
            .toAngularDistance(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicks().value

        front.set(ControlMode.MotionMagic, nativeSetpointFront)
    }

    private fun downwardsMoveBack(setpointBack: LinearDistanceMeasureInches) {
        //Configure motion velocities and accelerations
        val nativeVelocityBack = ControlParameters.ClimberParameters.climberVelocityDown
            .toAngularVelocity(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()

        val nativeAccelBack = ControlParameters.ClimberParameters.climberAccelerationDown
            .toAngularVelocity(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND

        back.master.configMotionCruiseVelocity(nativeVelocityBack, 0)
        back.master.configMotionAcceleration(nativeAccelBack, 0)

        //back.setPIDF(ControlParameters.ClimberParameters.BackDownPIDF, 0, 0)

        val nativeSetpointBack = setpointBack
            .toAngularDistance(Geometry.ClimberGeometry.backPitchRadius)
            .toMagEncoderTicks().value

        back.set(ControlMode.MotionMagic, nativeSetpointBack)
    }

    private fun downwardsMoveFront(setpointFront: LinearDistanceMeasureInches) {
        //Configure motion velocities and accelerations
        val nativeVelocityFront = ControlParameters.ClimberParameters.climberVelocityDown
            .toAngularVelocity(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt()

        val nativeAccelFront = ControlParameters.ClimberParameters.climberAccelerationDown
            .toAngularVelocity(Geometry.ClimberGeometry.frontPitchRadius)
            .toMagEncoderTicksPerHundredMilliseconds().value.roundToInt() //PER SECOND

        front.master.configMotionCruiseVelocity(nativeVelocityFront, 0)
        front.master.configMotionAcceleration(nativeAccelFront, 0)

        //front.setPIDF(ControlParameters.ClimberParameters.FrontDownPIDF, 0, 0)

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

    val climberMachine: StateMachine<ClimberStates> = stateMachine {
        state(ClimberStates.Disabled) {
            action {
                front.stop()
                back.stop()
            }
        }
        
        state(ClimberStates.TestDown) {
            entry {
                back.master.config_kF(0, SmartDashboard.getNumber("ffBack", 0.0), 0)
                front.master.config_kF(0, SmartDashboard.getNumber("ffFront", 0.0), 0)
                back.master.config_kP(0, SmartDashboard.getNumber("pBack", 0.0), 0)
                front.master.config_kP(0, SmartDashboard.getNumber("pFront", 0.0), 0)
                downwardsMoveBack(22.0.Inches)
                downwardsMoveFront(22.0.Inches)
            }

            action{
                println("Percent out : Front: ${front.master.motorOutputPercent}, Back : ${back.master.motorOutputPercent}")
            }

            exit {
                front.stop()
                back.stop()
            }
        }
        
        state (ClimberStates.Homing) {
            val backTicker = Ticker(
                { back.getVelocity() == 0.0.RadiansPerSecond },
                ControlParameters.ClimberParameters.homingTime,
                20.0.Milliseconds.toSeconds()
            )
            
            val frontTicker = Ticker(
                { front.getVelocity() == 0.0.RadiansPerSecond },
                ControlParameters.ClimberParameters.homingTime,
                20.0.Milliseconds.toSeconds()
            )
            
            var backDone = false
            var frontDone = false

            entry {
                backTicker.reset()
                frontTicker.reset()
                homed = false
                back.set(ControlParameters.ClimberParameters.homingPower)
                front.set(ControlParameters.ClimberParameters.homingPower)
                println("Homing climber")
            }

            action {
                backTicker.check {
                    //If we've stopped moving, and the time has elapsed, we're done
                    back.master.selectedSensorPosition = Geometry.ClimberGeometry.backHomeOffset
                        .toAngularDistance(Geometry.ClimberGeometry.backPitchRadius)
                        .toMagEncoderTicks().value.roundToInt()
                    
                    back.stop()
                    backDone = true
                }
                
                frontTicker.check {
                    front.master.selectedSensorPosition = Geometry.ClimberGeometry.frontHomeOffset
                        .toAngularDistance(Geometry.ClimberGeometry.frontPitchRadius)
                        .toMagEncoderTicks().value.roundToInt()
                    
                    front.stop()
                    frontDone = true
                }
                
                if (backDone && frontDone) {
                    homed = true
                    setState(ClimberStates.Stowed)
                }
            }

            exit {
                back.stop()
                front.stop()
                println("Climber homed")
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

            action {
                //Start checking our position.  If we're >= the setpoint - tolerance, move on.
                if (frontWithinTolerance(ControlParameters.ClimberPositions.l3Climb)
                    && backWithinTolerance(ControlParameters.ClimberPositions.l3Climb)) {
                    setState(ClimberStates.FallL3)
                }
            }
        }
        
        state (ClimberStates.FallL2) {
            rejectIf {
                !isInState(ClimberStates.DownL2) && !isInState(ClimberStates.LondonBridgeIsMaybeFallingDown)//Don't allow us to fall unless we're already in the right up state
            }

            entry {
                upwardsMoveBack(ControlParameters.ClimberPositions.stowed)
                downwardsMoveFront(ControlParameters.ClimberPositions.l2Climb)
            }
        }

        state (ClimberStates.FallL3) {
            rejectIf {
                !isInState(ClimberStates.DownL3) && !isInState(ClimberStates.LondonBridgeIsMaybeFallingDown) //Don't allow us to fall unless we're already in the right up state
            }

            entry {
                upwardsMoveBack(ControlParameters.ClimberPositions.stowed)
                downwardsMoveFront(ControlParameters.ClimberPositions.l3Climb)
            }
        }

        state(ClimberStates.SlowFall){
            entry {
                upwardsMoveBackSlow(ControlParameters.ClimberPositions.stowed)
                upwardsMoveFrontSlow(ControlParameters.ClimberPositions.stowed)
            }
        }

        state(ClimberStates.LondonBridgeIsMaybeFallingDown){
            rejectIf {
                !isInState(ClimberStates.FallL3) && !isInState(ClimberStates.FallL2)
            }

            entry {
                upwardsMoveFront(ControlParameters.ClimberPositions.stowed)
                upwardsMoveBack(ControlParameters.ClimberPositions.stowed)
            }
        }

        disabled {
            action {
                front.set(0.0)
                back.set(0.0)
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


        //debug
        //println("Back: ${back.getPosition().toLinearDistance(Geometry.ClimberGeometry.backPitchRadius).toInches()}\tFront: ${front.getPosition().toLinearDistance(Geometry.ClimberGeometry.frontPitchRadius).toInches()}")
        //println(climberMachine.getState())
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
        
        SmartDashboard.putNumber("ffBack", 0.0)
        SmartDashboard.putNumber("ffFront", 0.0)
        SmartDashboard.putNumber("pBack", 0.0)
        SmartDashboard.putNumber("pFront", 0.0)
    }
}