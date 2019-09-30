package org.team401.robot2019.vision2

import org.team401.taxis.geometry.Twist2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Pose2d
import kotlin.math.abs


object DifferentialKinematics {
    private const val EPSILON = 1.0e-9

    fun forwardKinematics(left_wheel_delta: Double, right_wheel_delta: Double): Twist2d {
        val deltaRotation =
            (right_wheel_delta - left_wheel_delta) / (VisionConstants.driveTrackWidthInches * VisionConstants.driveTrackScrubFactor)
        return forwardKinematics(left_wheel_delta, right_wheel_delta, deltaRotation)
    }

    fun forwardKinematics(left_wheel_delta: Double, right_wheel_delta: Double, delta_rotation_rads: Double): Twist2d {
        val dx = (left_wheel_delta + right_wheel_delta) / 2.0
        return Twist2d(dx, 0.0, delta_rotation_rads)
    }

    fun forwardKinematics(
        prev_heading: Rotation2d, left_wheel_delta: Double, right_wheel_delta: Double,
        current_heading: Rotation2d
    ): Twist2d {
        val dx = (left_wheel_delta + right_wheel_delta) / 2.0
        val dy = 0.0
        return Twist2d(dx, dy, prev_heading.inverse().rotateBy(current_heading).radians)
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous rotation.
     */
    fun integrateForwardKinematics(
        current_pose: Pose2d,
        forward_kinematics: Twist2d
    ): Pose2d {
        return current_pose.transformBy(Pose2d.exp(forward_kinematics))
    }

    /**
     * Uses inverse kinematics to convert a Twist2d into left and right wheel velocities
     */
    fun inverseKinematics(velocity: Twist2d): DriveSignal {
        if (abs(velocity.dtheta) < EPSILON) {
            return DriveSignal(velocity.dx, velocity.dx)
        }
        val deltaV = VisionConstants.driveTrackWidthInches * velocity.dtheta / (2 * VisionConstants.driveTrackScrubFactor)
        return DriveSignal(velocity.dx - deltaV, velocity.dx + deltaV)
    }
}