package org.team401.robot2019.control.superstructure.geometry

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.subsystems.WristSubsystem
import org.team401.robot2019.subsystems.arm.control.ArmKinematics

/**
 * Solves the positions of the end effectors of the robot, given state information about the rest of the system
 */

object EndpointKinematics {
    data class FullWristPose(
        val hatchEndpoint: Point2d,
        val hatchEndpointA: Point2d,
        val hatchEndpointB: Point2d,
        val cargoEndpoint: Point2d,
        val backstopEndpoint: Point2d
    ) {
        /**
         * Returns the largest distance away from the x axis
         *
         * @param side Positive number indicates that we should return the highest magnitude to the right, negative to the left
         */
        fun getHighestXMagnitude(side: Double): LinearDistanceMeasureInches {
            return if (side >= 0.0) {
                Math.max(cargoEndpoint.x.value, Math.max(hatchEndpointA.x.value, hatchEndpointB.x.value)).Inches
            } else {
                Math.min(cargoEndpoint.x.value, Math.min(hatchEndpointA.x.value, hatchEndpointB.x.value)).Inches
            }
        }
    }

    /**
     * Solves forward kinematics for the wrist, which returns an object containing the poses of all
     */
    fun forward(
        armState: ArmState,
        wristState: WristState,
        wristToolStates: WristSubsystem.WristToolStates
    ): FullWristPose {
        val armPose = ArmKinematics.forward(armState)

        val cargoToolLength = 0.0.Inches/*when (cargoGrabberState) {
            WristSubsystem.CargoGrabberStates.Unclamped -> Geometry.WristGeometry.pivotToCargoOpenEndpoint
            WristSubsystem.CargoGrabberStates.Clamped -> Geometry.WristGeometry.pivotToCargoClosedEndpoint
        }*/

        val hatchToolLength = 0.0.Inches/*when (hatchClawState) {
            WristSubsystem.HatchClawStates.Clamped -> Geometry.WristGeometry.pivotToHatchClamped
            WristSubsystem.HatchClawStates.Unclamped -> Geometry.WristGeometry.pivotToHatchOpen
        }*/

        val hatchToolHeight = 0.0.Inches/*when (hatchClawState) {
            WristSubsystem.HatchClawStates.Clamped -> Geometry.WristGeometry.hatchClawOpenHeight
            WristSubsystem.HatchClawStates.Unclamped -> 0.0.Inches
        }*/

        val wristAngleAdjusted = wristState.wristPosition - (Math.PI / 2.0).Radians + armState.armAngle
        val wristAngleNormal = wristAngleAdjusted + (Math.PI / 2.0).Radians

        val cargoEndpointX = armPose.x - (cargoToolLength.value * Math.cos(wristAngleAdjusted.value)).Inches
        val cargoEndpointY = armPose.y - (cargoToolLength.value * Math.sin(wristAngleAdjusted.value)).Inches

        val cargoBackstopX =
            armPose.x - (Geometry.WristGeometry.pivotToCargoBackstop.value * Math.cos(wristAngleAdjusted.value)).Inches
        val cargoBackstopY =
            armPose.y - (Geometry.WristGeometry.pivotToCargoBackstop.value * Math.sin(wristAngleAdjusted.value)).Inches

        val hatchEndpointX = armPose.x + (hatchToolLength.value * Math.cos(wristAngleAdjusted.value)).Inches
        val hatchEndpointY = armPose.y + (hatchToolLength.value * Math.sin(wristAngleAdjusted.value)).Inches

        val hatchEndpointAX = hatchEndpointX + (hatchToolHeight.value * Math.cos(wristAngleNormal.value)).Inches
        val hatchEndpointAY = hatchEndpointY + (hatchToolHeight.value * Math.sin(wristAngleNormal.value)).Inches

        val hatchEndpointBX = hatchEndpointX - (hatchToolHeight.value * Math.cos(wristAngleNormal.value)).Inches
        val hatchEndpointBY = hatchEndpointY - (hatchToolHeight.value * Math.sin(wristAngleNormal.value)).Inches

        val hatchEndpointPose = Point2d(hatchEndpointX, hatchEndpointY)
        val hatchEndpointAPose = Point2d(hatchEndpointAX, hatchEndpointAY)
        val hatchEndpointBPose = Point2d(hatchEndpointBX, hatchEndpointBY)
        val cargoEndpointPose = Point2d(cargoEndpointX, cargoEndpointY)
        val cargoBackstopPose = Point2d(cargoBackstopX, cargoBackstopY)

        return FullWristPose(
            hatchEndpointPose,
            hatchEndpointAPose,
            hatchEndpointBPose,
            cargoEndpointPose,
            cargoBackstopPose
        )
    }
}