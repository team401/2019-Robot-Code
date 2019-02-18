package org.team401.robot2019.control.superstructure.planning.profile

import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.PointPolar
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
    // TODO Use real numbers
    private val armProfile = TrapezoidalProfileGenerator(
        ControlParameters.ArmParameters.ROTATION_MAX_VELOCITY,
        ControlParameters.ArmParameters.ROTATION_MAX_ACCELERATION,
        startPoint.theta,
        endPoint.theta
    )

    fun solveSystem(): ArrayList<Pair<Point2d, PointPolar>>{
        //armProfile.generate()
        //armProfile.graph(true)
        val armRotation = armProfile.getPosition()

        val points = ArrayList<Pair<Point2d, PointPolar>>()
        println("Intersects circle: $intersectsCircle")
        armRotation.forEach {
            //println("Theta: $it, Endpos : ${endPoint.theta}")
            if (intersectsCircle) {
                if (startPoint.theta > endPoint.theta) {
                    when { // TODO Configure for forwards and backwards motion
                        ArmKinematics.inverse(segments[0].end).theta < it -> points.add(
                            Pair(
                                segments[0].solve(it),
                                ArmKinematics.inverse(segments[0].solve(it))
                            )
                        )
                        ArmKinematics.inverse(segments[1].end).theta < it -> points.add(
                            Pair(
                                segments[1].solve(it),
                                ArmKinematics.inverse(segments[1].solve(it))
                            )
                        )
                        ArmKinematics.inverse(segments[2].end).theta < it -> points.add(
                            Pair(
                                segments[2].solve(it),
                                ArmKinematics.inverse(segments[2].solve(it))
                            )
                        )
                    }
                }else {
                    // Switch for different types of function
                    when { // TODO Configure for forwards and backwards motion
                        ArmKinematics.inverse(segments[0].end).theta.value > it.value -> points.add(
                            Pair(
                                segments[0].solve(it),
                                ArmKinematics.inverse(segments[0].solve(it))
                            )
                        )
                        ArmKinematics.inverse(segments[1].end).theta > it -> points.add(
                            Pair(
                                segments[1].solve(it),
                                ArmKinematics.inverse(segments[1].solve(it))
                            )
                        )
                        ArmKinematics.inverse(segments[2].end).theta > it -> points.add(
                            Pair(
                                segments[2].solve(it),
                                ArmKinematics.inverse(segments[2].solve(it))
                            )
                        )
                    }
                }
            }else{
                points.add(Pair(segments[0].solve(it), ArmKinematics.inverse(segments[0].solve(it))))
            }
        }
        return points
    }

    fun solvePoint(theta: AngularDistanceMeasureRadians): Pair<Point2d, PointPolar>{
        //println("Theta: $theta, Endpos : ${endPoint.theta}")
        lateinit var point: Pair<Point2d, PointPolar>
        //println("Intersects circle: $intersectsCircle")

        if (intersectsCircle) {
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
                // Switch for different types of function
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
            //println("Theta : $theta")
            point = Pair(segments[0].solve(theta), ArmKinematics.inverse(segments[0].solve(theta)))
        }
    //println("theta: $theta")
    return point
    }

    fun getTime(): Array<TimeMeasureSeconds>{
        return armProfile.getTime()
    }

    private fun withinTolerance(value: Double, target: Double, tolerance: Double): Boolean{
        return abs(value - target) <= abs(tolerance)
    }

}