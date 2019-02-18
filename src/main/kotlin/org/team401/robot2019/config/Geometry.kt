package org.team401.robot2019.config

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.snakeskin.measure.MagEncoderTicks
import org.snakeskin.template.TankDrivetrainGeometryTemplate
import kotlin.math.atan2

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object Geometry {
    object DrivetrainGeometry: TankDrivetrainGeometryTemplate {
        override val wheelRadius = 3.062954.Inches
        override val wheelbase = 25.625.Inches
    }

    object ArmGeometry{
        //All values with the pivot as the origin, wrist pivot as the endpoint
        val maxX = 50.0.Inches
        val maxY = 100.0.Inches
        val minY = (-20.0).Inches
        val maxExtension = 0.0.Inches
        val maxArmLength = 0.0.Inches

        /**
         * Distance that the reference point on the extension sticks out from the base when fully retracted.
         * This is used when homing the extension to set the right offset distance.
         */
        val armExtensionStickout = 1.0.Inches

        /**
         * Length of the fixed part of the arm in inches, plus the minimum possible extension length
         * Essentially represents the distance from the arm pivot to the wrist pivot at minimum extension
         */
        val armBaseLength = 19.0.Inches + armExtensionStickout //Arm radius + minimum extension stickout

        /**
         * Minimum effective radius of the system that allows the wrist to rotate (i.e. not collide with the base)
         * Measured by rotating the longest side of the wrist towards the end of the arm
         */
        val minSafeArmLength = armBaseLength + 12.0.Inches + 1.5.Inches //Arm base + minimum distance + safety factor

        /**
         * Maximum distance down from the pivot that the wrist is safe to rotate at
         * Essentially lowest y coordinate that it is safe to rotate the wrist at
         */
        val minSafeWristRotationHeight = (-5.0).Inches

        /**
         * Minimum safe radius that it is safe to change tools at.  This allows an additional safety
         * factor for switching tools.
         */
        val minSafeWristToolChangeRadius = minSafeArmLength + 1.0.Inches

        /**
         * Linear range around the origin in x of the arm to swap sides when entered.
         * Essentially keeps the gamepiece from being crushed between the wrist and the arm itself when
         * moving while keeping the wrist parallel to the floor.
         */
        val wristParallelCollisionAngle = 12.0.Inches

        val pivotHeight = 0.0.Inches

        val maxTheta = atan2(maxY.value, maxX.value) // From -Pi to Pi

        val startExtension = 0.0.Inches // Distance from pivot to fully retracted

        /**
         * The "pitch radius" of the sprocket
         */
        val extensionPitchRadius = 0.720.Inches
    }

    object ClimberGeometry {
        /**
         * Pitch radius of the front gear, used to convert from angular position to linear position
         */
        val frontPitchRadius = 0.45.Inches

        /**
         * Pitch radius of the back gear, used to convert from angular position to linear position
         */
        val backPitchRadius = 0.45.Inches
    }
}