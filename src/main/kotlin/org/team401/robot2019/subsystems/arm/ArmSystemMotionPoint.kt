package org.team401.robot2019.subsystems.arm

import org.snakeskin.units.MagEncoderTicks
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder
import org.team401.robot2019.config.Geometry

data class ArmSystemMotionPoint(val targetExtension: AngularDistanceMeasureCTREMagEncoder,
                                val targetPosition: AngularDistanceMeasureCTREMagEncoder,
                                val targetWristPosition: AngularDistanceMeasureCTREMagEncoder,
                                val rotationFeedForward: Double){
    constructor(armState: ArmState, wristState: WristState, rotationFeedForward: Double): this((armState.armRadius - Geometry.ArmGeometry.minArmLength).toAngularDistance(Geometry.ArmGeometry.armToInches).toUnit(
        MagEncoderTicks) as AngularDistanceMeasureCTREMagEncoder,
        armState.armAngle.toUnit(MagEncoderTicks) as AngularDistanceMeasureCTREMagEncoder, wristState.wristPosition.toUnit(MagEncoderTicks) as AngularDistanceMeasureCTREMagEncoder, rotationFeedForward)
}

