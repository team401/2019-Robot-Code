package org.team401.robot2019.vision2

import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

/**
 * Extension of the limelight control class to allow triangulation of the vision targets in 3d space
 */
class LimelightCameraEnhanced(name: String, robotToCamera: Pose2d, val horizontalPlaneToCamera: Rotation2d, val height: Double): LimelightCamera(name, robotToCamera) {
    private val zeroArray = DoubleArray(8) { 0.0 }
    private val xCornersEntry = table.getEntry("tcornx")
    private val yCornersEntry = table.getEntry("tcorny")
    private val taEntry = table.getEntry("ta")

    private val constantLatencySeconds = constantLatency.toSeconds().value

    fun getLatencySeconds() = entries.tl.getDouble(0.0) / 1000.0 + constantLatencySeconds

    @Synchronized
    fun seesTarget() = entries.tv.getDouble(0.0) == 1.0

    @Synchronized
    fun getArea() = taEntry.getDouble(0.0)

}