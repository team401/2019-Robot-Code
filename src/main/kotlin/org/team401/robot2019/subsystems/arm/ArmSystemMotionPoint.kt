package org.team401.robot2019.subsystems.arm

import org.snakeskin.measure.distance.angular.AngularDistanceMeasureMagEncoderTicks
import org.team401.robot2019.config.Geometry

data class ArmSystemMotionPoint(val targetExtension: AngularDistanceMeasureMagEncoderTicks,
                                val targetPosition: AngularDistanceMeasureMagEncoderTicks,
                                val targetWristPosition: AngularDistanceMeasureMagEncoderTicks,
                                val rotationFeedForward: Double){
    constructor(armState: ArmState, wristState: WristState, rotationFeedForward: Double): this((armState.armRadius - Geometry.ArmGeometry.minArmLength).toAngularDistance(Geometry.ArmGeometry.armToInches).toMagEncoderTicks(),
        armState.armAngle.toMagEncoderTicks(), wristState.wristPosition.toMagEncoderTicks(), rotationFeedForward)
}

