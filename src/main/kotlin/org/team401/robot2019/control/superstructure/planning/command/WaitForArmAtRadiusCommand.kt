package org.team401.robot2019.control.superstructure.planning.command

import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.taxis.geometry.Pose2d

/**
 * @author Cameron Earle
 * @version 2/12/2019
 *
 * Command that blocks the queue until the arm is between a given range of radii
 *
 * @param lower The lower bound of allowed radii
 */
class WaitForArmAtRadiusCommand(val lower: LinearDistanceMeasureInches, val upper: LinearDistanceMeasureInches): SuperstructureCommand() {
    private var done = false

    override fun entry() {

    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState, drivePose: Pose2d) {
        val armRadius = armState.armRadius
        done = (armRadius > lower) && (armRadius < upper)
    }

    override fun isDone(): Boolean {
        return done
    }

    override fun getDescription(): String {
        return "Lower Bound: $lower | Upper Bound: $upper"
    }
}