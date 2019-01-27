package org.team401.robot2019.subsystems.arm

import org.team401.armsim.ArmKinematics
import org.team401.armsim.Point2d
import org.team401.armsim.PointPolar
import org.team401.armsim.TrapezoidalProfileGenerator
import org.team401.armsim.profile.ArmPath
import org.team401.armsim.profile.LinearProfileSegment
import org.team401.armsim.profile.Profile2d
import org.team401.armsim.profile.ProfileSegment
import org.team401.robot2019.config.ControlParameters

object ArmController{
    // 1. Calculate Path
    // 2. Calculate Point
    // 3. Find radius
    // 4.
    private var startPos = Point2d(Double.NaN, Double.NaN)
    private var endPos = Point2d(Double.NaN, Double.NaN)

    private var startTheta = ArmKinematics.inverse(ArmController.startPos).theta
    private var endTheta = ArmKinematics.inverse(ArmController.endPos).theta

    private var path = calculatePath()
    private var profile = Profile2d(path)
    private var rotationProfile = TrapezoidalProfileGenerator(
        ControlParameters.ArmParameters.MAX_VELOCITY,
        ControlParameters.ArmParameters.MAX_ACCELERATION,
        startTheta,
        endTheta
    )

    fun setDesiredPath(startPos: Point2d, endPos: Point2d){
        this.startPos = startPos
        this.endPos = endPos

        startTheta = ArmKinematics.inverse(ArmController.startPos).theta
        endTheta = ArmKinematics.inverse(ArmController.endPos).theta

        path = calculatePath()
        profile = Profile2d(path)
        rotationProfile = TrapezoidalProfileGenerator(
            ControlParameters.ArmParameters.MAX_VELOCITY,
            ControlParameters.ArmParameters.MAX_ACCELERATION,
            startTheta,
            endTheta
        )
    }

    fun update(): ArmState{
        val currentArmState = rotationProfile.updatePoint()
        val currentArmPosition = currentArmState[0]
        val currentArmVelocity = currentArmState[1]
        val currentRadius = profile.solvePoint(currentArmPosition).second.r

        return ArmState(PointPolar(currentRadius, currentArmPosition), currentArmVelocity)
    }

    private fun calculatePath(): Array<ProfileSegment>{
        val armPath = ArmPath(LinearProfileSegment(startPos, endPos))
        return armPath.solve()
    }

}