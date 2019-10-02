package org.team401.robot2019.vision2

import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d
import org.team401.taxis.util.Util
import org.team401.vision2.TargetInfo
import java.util.*
import kotlin.Comparator
import kotlin.math.abs

/**
 * Extension of the limelight control class to allow triangulation of the vision targets in 3d space
 */
class LimelightCameraEnhanced(name: String, robotToCamera: Pose2d, val horizontalPlaneToCamera: Rotation2d, val height: Double): LimelightCamera(name, robotToCamera) {
    private val zeroArray = DoubleArray(8) { 0.0 }
    private val xCornersEntry = table.getEntry("tcornx")
    private val yCornersEntry = table.getEntry("tcorny")
    private val taEntry = table.getEntry("ta")

    private var seesTarget = false
    private val targets = arrayListOf<TargetInfo>()

    private val constantLatencySeconds = constantLatency.toSeconds().value

    fun getLatencySeconds() = entries.tl.getDouble(0.0) / 1000.0 + constantLatencySeconds

    @Synchronized
    fun seesTarget() = entries.tv.getDouble(0.0) == 1.0

    @Synchronized
    fun getArea() = taEntry.getDouble(0.0)

    @Synchronized
    fun getTarget(): List<TargetInfo>? {
        val targets = getRawTargetInfos()
        if (seesTarget && targets != null) {
            return targets
        }

        return null
    }

    @Synchronized
    private fun getRawTargetInfos(): List<TargetInfo>? {
        val corners = getTopCorners() ?: return null

        var slope = 1.0
        if (abs(corners[1][0] - corners[0][0]) > Util.kEpsilon) {
            slope = (corners[1][1] - corners[0][1]) /
                    (corners[1][0] - corners[0][0])
        }

        targets.clear()
        for (i in 0 until 2) {
            //Average of y and z
            val yPixels = corners[i][0]
            val zPixels = corners[i][1]

            //Redefine to robot frame of reference
            val nY = -((yPixels - 160.0) / 160.0)
            val nZ = -((zPixels - 120.0) / 120.0)

            val y = VisionConstants.VPW / 2.0 * nY
            val z = VisionConstants.VPH / 2.0 * nZ

            val target = TargetInfo(y, z)
            target.skew = slope
            targets.add(target)
        }
        return targets
    }

    private fun getTopCorners(): List<DoubleArray>? {
        val xCorners = xCornersEntry.getDoubleArray(zeroArray)
        val yCorners = yCornersEntry.getDoubleArray(zeroArray)
        seesTarget = entries.tv.getDouble(0.0) == 1.0

        if (!seesTarget ||
                Arrays.equals(xCorners, zeroArray) || Arrays.equals(yCorners, zeroArray)
            || xCorners.size != 8 || yCorners.size != 8) {
            return null
        }

        return extractTopCornersFromBoundingBoxes(xCorners, yCorners)
    }

    companion object {
        private val xSort = Comparator.comparingDouble(Translation2d::x)
        private val ySort = Comparator.comparingDouble(Translation2d::y)

        fun extractTopCornersFromBoundingBoxes(xCorners: DoubleArray, yCorners: DoubleArray): List<DoubleArray> {
            val corners = arrayListOf<Translation2d>()
            for (i in xCorners.indices) {
                corners.add(Translation2d(xCorners[i], yCorners[i]))
            }

            corners.sortWith(xSort)

            val left = corners.subList(0, 4)
            val right = corners.subList(4, 8)

            left.sortWith(ySort)
            right.sortWith(ySort)

            val leftTop = left.subList(0, 2)
            val rightTop = right.subList(0, 2)

            leftTop.sortWith(xSort)
            rightTop.sortWith(xSort)

            val leftCorner = leftTop[0]
            val rightCorner = rightTop[1]

            return listOf(
                doubleArrayOf(leftCorner.x(), leftCorner.y()),
                doubleArrayOf(rightCorner.x(), rightCorner.y())
            )
        }
    }
}