package org.team401.armsim.profile

import org.snakeskin.units.RadiansPerSecond
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.time.TimeMeasureSeconds
import org.team401.armsim.ArmKinematics
import org.team401.armsim.Point2d
import org.team401.armsim.PointPolar
import org.team401.armsim.TrapezoidalProfileGenerator
import org.team401.robot2019.config.ControlParameters


class Profile2d(private val segments: Array<ProfileSegment>) {
    // Find the two end points for arm profile generation
    private val startPoint = ArmKinematics.inverse(segments.first().start)
    private val endPoint = ArmKinematics.inverse(segments.last().end)

    private val intersectsCircle = segments.size > 1
    // TODO Use real numbers
    private val armProfile = TrapezoidalProfileGenerator(3.14.RadiansPerSecond, 3.14.RadiansPerSecond, startPoint.theta, endPoint.theta)

    fun solveSystem(): ArrayList<Pair<Point2d, PointPolar>>{
        //armProfile.generate()
        //armProfile.graph(true)
        val armRotation = armProfile.getPosition()

        val points = ArrayList<Pair<Point2d, PointPolar>>()
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
        var point = Pair(ControlParameters.ArmParameters.DEFAULT_ARM_POSITION, ArmKinematics.inverse(ControlParameters.ArmParameters.DEFAULT_ARM_POSITION))
        if (intersectsCircle) {
            if (startPoint.theta > endPoint.theta) {
                when { // TODO Configure for forwards and backwards motion
                    ArmKinematics.inverse(segments[0].end).theta < theta -> {
                        point = Pair(segments[0].solve(theta), ArmKinematics.inverse(segments[0].solve(theta)))
                    }
                    ArmKinematics.inverse(segments[1].end).theta < theta -> {
                        point = Pair(segments[1].solve(theta),ArmKinematics.inverse(segments[1].solve(theta)))
                    }

                    ArmKinematics.inverse(segments[2].end).theta < theta -> {
                        point = Pair(segments[2].solve(theta),ArmKinematics.inverse(segments[2].solve(theta)))
                    }
                }
            }else {
                // Switch for different types of function
                when { // TODO Configure for forwards and backwards motion
                    ArmKinematics.inverse(segments[0].end).theta > theta -> {
                        point = Pair(segments[0].solve(theta),ArmKinematics.inverse(segments[0].solve(theta)))
                    }
                    ArmKinematics.inverse(segments[1].end).theta > theta -> {
                        point = Pair(segments[1].solve(theta),ArmKinematics.inverse(segments[1].solve(theta)))
                    }
                    ArmKinematics.inverse(segments[2].end).theta > theta -> {
                        point = Pair(segments[2].solve(theta),ArmKinematics.inverse(segments[2].solve(theta)))
                    }
                }
            }
        }else{
            point = Pair(segments[0].solve(theta), ArmKinematics.inverse(segments[0].solve(theta)))
        }

    return point
    }

    fun getTime(): Array<TimeMeasureSeconds>{
        return armProfile.getTime()
    }

}