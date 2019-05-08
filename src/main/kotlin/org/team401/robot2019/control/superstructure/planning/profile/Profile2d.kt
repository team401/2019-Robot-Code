package org.team401.robot2019.control.superstructure.planning.profile

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import kotlin.math.abs

/**
 * @author Eli Jelesko
 * @version 1/19/2019
 *
 */
class Profile2d(private val segments: Array<ProfileSegment>) {
    // Find the two end points for superstructure profile generation
    private val startPoint = ArmKinematics.inverse(segments.first().start)
    private val endPoint = ArmKinematics.inverse(segments.last().end)

    private val intersectsCircle = segments.size > 1
    private val armProfile = TrapezoidalProfileGenerator(
        ControlParameters.ArmParameters.rotationVelocity,
        ControlParameters.ArmParameters.rotationAcceleration,
        startPoint.theta,
        endPoint.theta
    )

    fun solvePoint(theta: AngularDistanceMeasureRadians): Pair<Point2d, PointPolar>{
        lateinit var point: Pair<Point2d, PointPolar>

        // Find if the desired motion would go through an illegal position
        if (intersectsCircle) {
            // Check to see if the system is running "backwards" (going from a larger theta value to a smaller one)
            if (startPoint.theta > endPoint.theta) {
                when {
                    ArmKinematics.inverse(segments[0].end).theta < theta -> {
                        point = Pair(segments[0].solve(theta), ArmKinematics.inverse(segments[0].solve(theta)))
                    }
                    ArmKinematics.inverse(segments[1].end).theta < theta -> {
                        point = Pair(segments[1].solve(theta), ArmKinematics.inverse(segments[1].solve(theta)))
                    }

                    ArmKinematics.inverse(segments[2].end).theta <= theta -> {
                        point = Pair(segments[2].solve(theta), ArmKinematics.inverse(segments[2].solve(theta)))
                    }
                }
            }else {
                // Adjust is the system is running "forwards"
                when {
                    ArmKinematics.inverse(segments[0].end).theta > theta -> {
                        point = Pair(segments[0].solve(theta), ArmKinematics.inverse(segments[0].solve(theta)))
                    }
                    ArmKinematics.inverse(segments[1].end).theta > theta -> {
                        point = Pair(segments[1].solve(theta), ArmKinematics.inverse(segments[1].solve(theta)))
                    }
                    ArmKinematics.inverse(segments[2].end).theta >= theta -> {
                        point = Pair(segments[2].solve(theta), ArmKinematics.inverse(segments[2].solve(theta)))
                    }
                }
            }
        }else{
            // If the desired motion is legal, return that path
            point = Pair(segments[0].solve(theta), ArmKinematics.inverse(segments[0].solve(theta)))
        }
    return point
    }

    fun getTime(): Array<TimeMeasureSeconds>{
        return armProfile.getTime()
    }

    private fun withinTolerance(value: Double, target: Double, tolerance: Double): Boolean{
        return abs(value - target) <= abs(tolerance)
    }

}