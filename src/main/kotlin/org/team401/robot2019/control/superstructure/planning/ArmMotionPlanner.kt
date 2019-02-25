package org.team401.robot2019.control.superstructure.planning

import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.planning.profile.TrapezoidalProfileGenerator
import org.team401.robot2019.control.superstructure.planning.profile.ArmPath
import org.team401.robot2019.control.superstructure.planning.profile.LinearProfileSegment
import org.team401.robot2019.control.superstructure.planning.profile.Profile2d
import org.team401.robot2019.control.superstructure.planning.profile.ProfileSegment
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.planning.profile.TrapezoidalProfilePoint
/**
 * @author Eli Jelesko
 * @version 1/19/2019
 *
 */
object ArmMotionPlanner{
    // 1. Calculate Path
    // 2. Calculate Point
    // 3. Find radius
    // 4.
    private lateinit var startPos: Point2d
    private lateinit var endPos: Point2d

    private var startTheta = 0.0.Radians
    private var endTheta = 0.0.Radians

    private var currentTime = 0.0.Seconds

    private lateinit var path: Array<ProfileSegment>
    private lateinit var profile: Profile2d
    private lateinit var rotationProfile: TrapezoidalProfileGenerator

    private var done = false

    fun setDesiredTrajectory(startPos: Point2d, endPos: Point2d, minimumRadius: LinearDistanceMeasureInches){
        reset()
        ArmMotionPlanner.startPos = startPos
        ArmMotionPlanner.endPos = endPos

        startTheta = ArmKinematics.inverse(ArmMotionPlanner.startPos).theta
        endTheta = ArmKinematics.inverse(ArmMotionPlanner.endPos).theta

        path = calculatePath(minimumRadius)
        profile = Profile2d(path)
        rotationProfile = TrapezoidalProfileGenerator(
            ControlParameters.ArmParameters.ROTATION_MAX_VELOCITY,
            ControlParameters.ArmParameters.ROTATION_MAX_ACCELERATION,
            startTheta,
            endTheta
        )
    }

    fun reset(){
        done = false
    }

    fun isDone(): Boolean {
        return done
    }

    fun getCurrentTime(): TimeMeasureSeconds {
        return currentTime
    }

    fun update(dt: Double): ArmState {
        val currentArmState: TrapezoidalProfilePoint
        val currentArmPosition: AngularDistanceMeasureRadians
        val currentArmVelocity: AngularVelocityMeasureRadiansPerSecond
        val currentRadius: LinearDistanceMeasureInches

        if (Math.abs(startTheta.value - endTheta.value) < 0.01) {
            currentArmPosition = startTheta
            currentArmVelocity = 0.0.RadiansPerSecond
            currentRadius = ArmKinematics.inverse(endPos).r
            currentTime = 0.0.Seconds // TODO See if this causes problems
            done = true
        } else {
            currentArmState = rotationProfile.update(dt)
            currentArmPosition = currentArmState.position
            currentArmVelocity = currentArmState.velocity
            currentRadius = profile.solvePoint(currentArmPosition).second.r
            currentTime = currentArmState.time

            done = rotationProfile.isDone()
        }

        return ArmState(
            currentRadius,
            currentArmPosition,
            currentArmVelocity
        )
    }

    private fun calculatePath(minimumRadius: LinearDistanceMeasureInches): Array<ProfileSegment>{
        val armPath = ArmPath(
            LinearProfileSegment(startPos, endPos),
            //Geometry.ArmGeometry.minSafeArmLength
            minimumRadius
        )
        //println("start: $startPos, end: $endPos")
        return armPath.solve()
    }

}