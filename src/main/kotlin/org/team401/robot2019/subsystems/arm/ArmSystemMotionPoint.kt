package org.team401.robot2019.subsystems.arm

import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureCTREMagEncoder

data class ArmSystemMotionPoint(val targetExtension: AngularDistanceMeasureCTREMagEncoder,
                                val targetRotation: AngularDistanceMeasureCTREMagEncoder,
                                val rotationFeedForward: Double)