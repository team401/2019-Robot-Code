package org.team401.robot2019.subsystems.arm

import org.snakeskin.units.MagEncoderTicks
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.units.measure.time.TimeMeasureSeconds
import org.team401.armsim.ArmKinematics
import org.team401.armsim.Point2d
import org.team401.armsim.TrapezoidalProfileGenerator
import org.team401.armsim.profile.ArmPath
import org.team401.armsim.profile.LinearProfileSegment
import org.team401.armsim.profile.Profile2d
import org.team401.armsim.profile.ProfileSegment
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry

object ArmPather{
    // 1. Calculate Path
    // 2. Calculate Point
    // 3. Find radius
    // 4.
    private lateinit var startPos: Point2d
    private lateinit var endPos: Point2d

    private lateinit var startTheta: AngularDistanceMeasureRadians
    private lateinit var endTheta: AngularDistanceMeasureRadians

    private lateinit var currentTime: TimeMeasureSeconds

    private lateinit var path: Array<ProfileSegment>
    private lateinit var profile: Profile2d
    private lateinit var rotationProfile: TrapezoidalProfileGenerator
            /*= TrapezoidalProfileGenerator(
        ControlParameters.ArmParameters.MAX_VELOCITY,
        ControlParameters.ArmParameters.MAX_ACCELERATION,
        startTheta,
        endTheta
    )*/

    private var done = false

    fun setDesiredPath(startPos: Point2d, endPos: Point2d){
        this.startPos = startPos
        this.endPos = endPos

        startTheta = ArmKinematics.inverse(ArmPather.startPos).theta
        endTheta = ArmKinematics.inverse(ArmPather.endPos).theta

        path = calculatePath()
        profile = Profile2d(path)
        rotationProfile = TrapezoidalProfileGenerator(
            ControlParameters.ArmParameters.MAX_VELOCITY,
            ControlParameters.ArmParameters.MAX_ACCELERATION,
            startTheta,
            endTheta
        )
    }

    fun reset(){
        done = false
    }

    fun isDone(): Boolean{
        return done
    }

    fun getCurrentTime(): TimeMeasureSeconds{
        return currentTime
    }

    fun update(): ArmState{
        val currentArmState = rotationProfile.updatePoint()
        val currentArmPosition = currentArmState.position
        val currentArmVelocity = currentArmState.velocity
        val currentRadius = profile.solvePoint(currentArmPosition).second.r
        val currentExtension = (currentRadius - Geometry.ArmGeometry.minArmLength) as LinearDistanceMeasureInches
        currentTime = currentArmState.time

        done = rotationProfile.isDone()

        return ArmState(Pair(currentExtension, currentArmPosition), currentArmVelocity)
    }

    private fun calculatePath(): Array<ProfileSegment>{
        val armPath = ArmPath(LinearProfileSegment(startPos, endPos))
        return armPath.solve()
    }

}