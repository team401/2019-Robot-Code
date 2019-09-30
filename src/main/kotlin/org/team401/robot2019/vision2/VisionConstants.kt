package org.team401.robot2019.vision2

import kotlin.math.tan

object VisionConstants {
    const val lowTargetHeight = 31.5 //inches
    const val highTargetHeight = 39.125 //inches

    const val maxGoalTrackAge = 2.5
    const val maxTrackerDistance = 9.0
    const val maxGoalTrackSmoothingTime = 0.5
    const val cameraFrameRate = 90.0

    const val trackStabilityWeight = 0.0
    const val trackAgeWeight = 10.0
    const val trackSwitchingWeight = 100.0

    const val horizontalFOV = 59.6 //degrees
    const val verticalFOV = 49.7 //degrees
    val VPW = 2.0 * tan(Math.toRadians(horizontalFOV / 2.0))
    val VPH = 2.0 * tan(Math.toRadians(verticalFOV / 2.0))

    const val driveTrackWidthInches = 1.0
    const val driveTrackScrubFactor = 1.0

    const val driveWheelRadiusInches = 3.0
}