package org.team401.robot2019.control.climbing

import org.snakeskin.measure.Inches
import org.snakeskin.measure.InchesPerSecondPerSecond
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.config.ControlParameters

/**
 * This acts as both a controller and a "motion planner" for the climbing system.
 * We will only use this controller for the initial lift.  All other operations
 * (i.e. operations that only move a single leg) will just use motion magic.
 */
object ClimbingController {
    private lateinit var activeProfileBack: ClimbingProfileGenerator
    private lateinit var activeProfileFront: ClimbingProfileGenerator

    /**
     * Updates the climber controller.
     *
     * @param dt The delta time between calls
     * This should be positive when the front of the robot is angled up when viewed from the right side.
     * @return The target state of the climber
     */
    @Synchronized fun update(dt: Double): ClimberState {
        val referenceOutputBack = activeProfileBack.update(dt)
        val referenceOutputFront = activeProfileFront.update(dt)

        return ClimberState(referenceOutputBack.position, referenceOutputFront.position)
    }

    /**
     * Sets the setpoint for the climber system
     *
     * @param currentState The starting point, should just be read from sensors
     * @param target The setpoint
     *
     * This expects both legs to be in (about) the same position when starting a profile
     */
    @Synchronized fun setSetpoint(currentState: ClimberState, target: LinearDistanceMeasureInches) {
        //Capture the climber that is furthest down, ues it as the starting position.
        //This helps to synchronize the profile by quickly snapping the "lagging" leg
        //into the position that it is supposed to be in
        val maxStart = Math.max(currentState.backPosition.value, currentState.frontPosition.value).Inches

        activeProfileBack = ClimbingProfileGenerator(
                ControlParameters.ClimberParameters.climberVelocityDownClimbBack,
            ControlParameters.ClimberParameters.climberAccelerationDown.value.InchesPerSecondPerSecond,
            maxStart,
            target
        )

        activeProfileFront = ClimbingProfileGenerator(
            ControlParameters.ClimberParameters.climberVelocityDownClimbFront,
            ControlParameters.ClimberParameters.climberAccelerationDown.value.InchesPerSecondPerSecond,
            maxStart,
            target
        )
    }

    /**
     * Returns true when the profile has finished executing
     */
    @Synchronized fun isDone(): Boolean {
        return activeProfileBack.isDone() && activeProfileFront.isDone()
    }
}