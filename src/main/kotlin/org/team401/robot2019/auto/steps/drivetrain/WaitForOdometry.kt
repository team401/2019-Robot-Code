package org.team401.robot2019.auto.steps.drivetrain

import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.subsystems.DrivetrainSubsystem

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 * Waits for the odometry (estimated, not from vision) to cross a certain threshold.  The step will unblock
 * when the field to robot pose surpasses the provided threshold, along the provided axis, in the provided direction
 */
class WaitForOdometry(val axis: Axis, val direction: Direction, val threshold: Double): AutoStep() {
    enum class Axis {
        X,
        Y,
        THETA
    }

    enum class Direction {
        POSITIVE,
        NEGATIVE
    }

    override fun entry(currentTime: Double) {

    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        val translation = DrivetrainSubsystem.driveState.getFieldToVehicle(currentTime).translation
        val rotation = DrivetrainSubsystem.driveState.getFieldToVehicle(currentTime).rotation
        return when (axis) {
            Axis.X -> {
                when (direction) {
                    Direction.POSITIVE -> translation.x() >= threshold
                    Direction.NEGATIVE -> translation.x() <= threshold
                }
            }

            Axis.Y -> {
                when (direction) {
                    Direction.POSITIVE -> translation.y() >= threshold
                    Direction.NEGATIVE -> translation.y() <= threshold
                }
            }

            Axis.THETA -> {
                when (direction) {
                    Direction.POSITIVE -> rotation.degrees >= threshold
                    Direction.NEGATIVE -> rotation.degrees <= threshold
                }
            }
        }
    }

    override fun exit(currentTime: Double) {

    }
}