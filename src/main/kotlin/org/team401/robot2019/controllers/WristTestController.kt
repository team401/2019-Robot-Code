package org.team401.robot2019.controllers

import org.snakeskin.units.*
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasure
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians

/**
 * @author Cameron Earle
 * @version 1/11/2019
 *
 */
object WristTestController {
    private val k2Pi = Math.PI * 2.0

    private val kWristLagDelta = 45.Degrees.toUnit(Radians) //Angular error at which to restart the profile
    private val kCruiseVelocity = 61.RadiansPerSecond
    private val kAcceleration = (61.0 * 4.0).RadiansPerSecond //Per second


    private var finalGoal: AngularDistanceMeasure = 0.Radians
    private var direction: Boolean = false //false is cw, true is ccw

    private var wristTheta = 0.Radians
    private var activeProfilePosition = 0.Radians
    private var activeProfileLastPosition = 0.Radians
    private var activeProfileVelocity = 0.RadiansPerSecond
    private var activeProfileRelativeGoal = 0.Radians
    private var activeProfileHalfwayPoint = 0.Radians
    private var activeProfileCruiseTimeAccum = 0.0
    private var profileRunning = false

    /**
     * Calculates the relative distance to the target.  Takes a starting position, a target, and a direction.
     * true indicates angle on a unit circle increasing (counterclockwise)
     */
    fun calculateAbsoluteDistanceToTarget(start: AngularDistanceMeasure, target: AngularDistanceMeasure, direction: Boolean): AngularDistanceMeasure {
        val distance = Math.abs(target.toUnit(Radians).value - start.toUnit(Radians).value)
        return if (direction) {
            (distance % k2Pi).Radians
        } else {
            ((k2Pi - distance) % k2Pi).Radians
        }
    }

    /**
     * Sets the target for the controller.  This is a setpoint angle, as well as a direction to get there.
     * true is ccw, false is cw.
     * Depending on the magnitude of the setpoint change, this could cause a new profile to begin
     */
    @Synchronized
    fun setSetpoint(setpoint: AngularDistanceMeasure, targetDirection: Boolean) {
        finalGoal = setpoint.toUnit(Radians)
        direction = targetDirection
    }

    /**
     * Accepts the current state of the wrist and the current timestep
     * as well as an output array which will be populated in the format [position_setpoint, velocity_ff_setpoint]
     */
    @Synchronized
    fun update(wristAngle: AngularDistanceMeasure, dt: Double, outputArray: DoubleArray) {
        wristTheta = wristAngle.toUnit(Radians) as AngularDistanceMeasureRadians //Update wrist angle

        /**
         * If the error is greater than the acceptable value, and a profile isn't already running, start a new one
         */
        if (((finalGoal - wristTheta).value >= kWristLagDelta.value) && !profileRunning) {
            println("Error too great, restarting profile!")

            profileRunning = true
            //Configure parameters of profile.  We want to do everything in the relative, positive space to keep the generator simple
            activeProfilePosition = 0.Radians
            activeProfileLastPosition = 0.Radians
            activeProfileVelocity = 0.RadiansPerSecond
            activeProfileRelativeGoal = calculateAbsoluteDistanceToTarget(wristTheta, finalGoal, direction).toUnit(Radians) as AngularDistanceMeasureRadians //The absolute distance to our target
            activeProfileHalfwayPoint = (activeProfileRelativeGoal.toUnit(Radians).value / 2.0).Radians
            activeProfileCruiseTimeAccum = 0.0
        }

        if (profileRunning) {
            //Update profile
            if (activeProfilePosition.value < activeProfileHalfwayPoint.value) {
                activeProfileVelocity = Math.min(kCruiseVelocity.value, activeProfileVelocity.value + (kAcceleration.value * dt)).RadiansPerSecond
                if (activeProfileVelocity.value == kCruiseVelocity.value) {
                    activeProfileCruiseTimeAccum += dt
                }
            } else {
                activeProfileCruiseTimeAccum -= dt
                if (activeProfileCruiseTimeAccum <= 0.0) {
                    activeProfileVelocity = Math.max(0.0, activeProfileVelocity.value - (kAcceleration.value * dt)).RadiansPerSecond
                }
            }
            activeProfilePosition = Math.min(activeProfileRelativeGoal.value, activeProfileLastPosition.value + (activeProfileVelocity.value * dt)).Radians

            if (activeProfileVelocity.value == 0.0 && activeProfilePosition != activeProfileRelativeGoal) {
                activeProfilePosition = activeProfileRelativeGoal
            }

            //Check if done
            if (Math.abs(activeProfilePosition.value - activeProfileRelativeGoal.value) > 0) {
                profileRunning = false
            }

            activeProfileLastPosition = activeProfilePosition
        }

        outputArray[0] = activeProfilePosition.toUnit(MagEncoderTicks).value
        outputArray[1] = activeProfileVelocity.toUnit(MagEncoderTicksPer100Ms).value
    }
}

fun main(args: Array<String>) {
    println(WristTestController.calculateAbsoluteDistanceToTarget((180.0).Degrees, (-180.0).Degrees, false).toUnit(Degrees).value)

}