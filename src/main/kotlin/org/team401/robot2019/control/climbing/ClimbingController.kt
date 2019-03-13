package org.team401.robot2019.control.climbing

import org.snakeskin.measure.Inches
import org.snakeskin.measure.InchesPerSecondPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.config.ControlParameters

/**
 * This acts as both a controller and a "motion planner" for the climbing system.
 * We will only use this controller for the initial lift.  All other operations
 * (i.e. operations that only move a single leg) will just use motion magic.
 */
object ClimbingController {
    private lateinit var activeProfileBack: ClimbingProfileGenerator
    private lateinit var activeProfileFront: ClimbingProfileGenerator //TODO do better handling of these, create in update

    val kP = 0.0 //TODO move to constants

    /**
     * Updates the climber controller.
     *
     * @param dt The delta time between calls
     * @param chassisPitch The angle of the chassis to the floor.
     * This should be positive when the front of the robot is angled up when viewed from the right side.
     * @return The target state of the climber
     */
    @Synchronized fun update(dt: Double, chassisPitch: AngularDistanceMeasureRadians): ClimberState {
        val backOutput = activeProfileBack.update(dt)
        val frontOutput = activeProfileFront.update(dt)
        val angleAdjustment = chassisPitch.value * kP
        //A positive angle means we want to increase the setpoint of the back legs, and decrease the front
        val backSetpointFinal = backOutput.position + angleAdjustment.Inches
        val frontSetpointFinal = frontOutput.position - angleAdjustment.Inches

        return ClimberState(backSetpointFinal, frontSetpointFinal)
    }

    /**
     * Sets the setpoint for the climber system
     *
     * @param currentState The starting point, should just be read from sensors
     * @param target The setpoint
     *
     * This expects both legs to be in the same position when starting a profile
     */
    @Synchronized fun setSetpoint(currentState: ClimberState, target: LinearDistanceMeasureInches) {
        activeProfileBack = ClimbingProfileGenerator(
            ControlParameters.ClimberParameters.climberVelocityDown,
            ControlParameters.ClimberParameters.climberAccelerationDown.value.InchesPerSecondPerSecond, //TODO fix in constants
            currentState.backPosition,
            target
        )

        activeProfileFront = ClimbingProfileGenerator(
            ControlParameters.ClimberParameters.climberVelocityDown,
            ControlParameters.ClimberParameters.climberAccelerationDown.value.InchesPerSecondPerSecond, //TODO fix in constants
            currentState.frontPosition,
            target
        )
    }
}