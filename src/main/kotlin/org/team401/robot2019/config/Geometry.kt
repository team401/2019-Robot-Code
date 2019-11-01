package org.team401.robot2019.config

import org.snakeskin.measure.Centimeters
import org.snakeskin.measure.Inches
import org.snakeskin.template.TankDrivetrainGeometryTemplate
import org.snakeskin.utility.Selectable
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import kotlin.math.atan2

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object Geometry {
    /**
     * Geometry constants for the drivetrain
     */
    object DrivetrainGeometry: TankDrivetrainGeometryTemplate {
        override val wheelRadius = 3.00586.Inches
        override val wheelbase = 26.5.Inches

        /**
         * Distance in inches from the center of the robot to the frontmost edge, including bumpers
         */
        val robotHalfLength = 18.0.Inches

        /**
         * Distance in inches from the center of the robot to the leftmost edge, including bumpers
         */
        val robotHalfWidth = 18.0.Inches

        /**
         * Gear ratio of the high gear, used to convert wheel speeds to motor speeds at the NEO for velocity PID control
         */
        const val gearRatioHigh = (50.0/28.0) * (64.0 / 15.0)
    }

    /**
     * Geometry constants for the arm
     */
    object ArmGeometry {
        //All values with the pivot as the origin, wrist pivot as the endpoint
        val maxX = 70.0.Inches
        val maxY = 70.0.Inches
        val minY = (-30.0).Inches
        val maxExtension = 0.0.Inches
        val maxArmLength = 0.0.Inches


        /**
         * Distance that the reference point on the extension sticks out from the base when fully retracted.
         * This is used when homing the extension to set the right offset distance.
         */
        val armExtensionStickout = 2.125.Inches

        /**
         * Length of the fixed part of the arm in inches, plus the minimum possible extension length
         * Essentially represents the distance from the arm pivot to the wrist pivot at minimum extension
         */
        val armBaseLength = 19.0.Inches //Arm radius + minimum extension stickout

        /**
         * Minimum effective radius of the system that allows the wrist to rotate (i.e. not collide with the base)
         * Measured by rotating the longest side of the wrist towards the end of the arm
         */

        val hatchPanelToolMinSafeLength = armBaseLength + armExtensionStickout + 1.0.Inches
        val cargoToolMinSafeLength = armBaseLength + armExtensionStickout + 1.0.Inches
        val minSafeArmLength = cargoToolMinSafeLength//armBaseLength + armExtensionStickout + 12.0.Inches + 1.5.Inches //Arm base + minimum distance + safety factor


        /**
         * Maximum distance down from the pivot that the wrist is safe to rotate at
         * Essentially lowest y coordinate that it is safe to rotate the wrist at
         */
        val minSafeWristRotationHeight = (-15.0).Inches

        /**
         * Minimum safe radius that it is safe to change tools at.  This allows an additional safety
         * factor for switching tools.
         */
        val minSafeWristToolChangeRadius = minSafeArmLength

        val minToolChangeX = minSafeWristToolChangeRadius
        val minToolChangeY = 0.0.Inches

        /**
         * Linear range around the origin in x of the arm to swap sides when entered.
         * Essentially keeps the gamepiece from being crushed between the wrist and the arm itself when
         * moving while keeping the wrist parallel to the floor.
         */
        val wristParallelCollisionAngle = 12.0.Inches

        /**
         * Distance from the origin to the edge of the frame, in the positive direction.  This is symmetric.
         */
        val originToFrame = 14.5.Inches

        /**
         * Distance from the origin to the flor, in the negative direction
         */
        val originToFloor = (-23.0).Inches

        val maxTheta = atan2(maxY.value, maxX.value) // From -Pi to Pi

        val startExtension = 0.0.Inches // Distance from pivot to fully retracted

        /**
         * The "pitch radius" of the sprocket
         */
        val extensionPitchRadius = 0.720.Inches

        /**
         * Maximum legal x coordinate for a component of the wrist.
         */
        val maximumLegalWristX = originToFrame + 30.0.Inches
    }

    object WristGeometry {
        val pivotToCargoBackstop = 5.0.Inches
        val pivotToCargoClosedEndpoint = 13.0.Inches
        val pivotToCargoOpenEndpoint = 9.0.Inches
        val pivotToHatchClamped = 10.0.Inches
        val pivotToHatchOpen = 12.0.Inches
        val hatchClawOpenHeight = 5.0.Inches
    }

    /**
     * Geometry of the climber
     */
    object ClimberGeometry {
        /**
         * Pitch radius of the front gear, used to convert from angular position to linear position
         */
        val frontPitchRadius = 0.45.Inches

        /**
         * Pitch radius of the back gear, used to convert from angular position to linear position
         */
        val backPitchRadius = 0.45.Inches

        /**
         * Distance from the ground that the legs are at when homed.  These are used to offset the home position
         * so that zero is right on the ground.  These constants should be negative
         */
        val frontHomeOffset = (-0.5).Inches
        val backHomeOffset = (-1.25).Inches
    }

    /**
     * Geometry of the cameras with respect to the center of the drivetrain
     */
    object VisionGeometry {
        //Cameras are 8 inches up from the pivot
        val robotToFrontCamera = Pose2d(1.75, 6.0, Rotation2d.fromDegrees(0.0))
        val robotToBackCamera = Pose2d(-0.5, 6.0, Rotation2d.fromDegrees(180.0))
    }
}