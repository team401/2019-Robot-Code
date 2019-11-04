package org.team401.robot2019.vision2

import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.geometry.VisionHeightMode
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Translation2d
import kotlin.math.tan

object VisionSolver {
    //Regressions
    private val hatchStationFrontRegression = VisionRegression(83.699302, -0.492193)
    private val hatchStationBackRegression = VisionRegression(83.699302, -0.492193)
    private val hatchScoringFrontRegression = VisionRegression(84.951478, -0.490044, -1.0)
    private val hatchScoringAutoFrontRegression = VisionRegression(84.951478, -0.490044, -1.0)
    private val hatchScoringBackRegression = VisionRegression(84.951478, -0.490044, -1.0)
    private val cargoScoringRocketFrontRegression = VisionRegression(84.951478, -0.490044)
    private val cargoScoringRocketBackRegression = cargoScoringRocketFrontRegression

    private var activeRegression = hatchStationFrontRegression

    /**
     * Selects the correct regression to use given the conditions
     */
    fun selectRegression(activeSide: SuperstructureRoutines.Side, activeMode: VisionHeightMode, isAuto: Boolean = false) {
        activeRegression = when (activeSide) {
            SuperstructureRoutines.Side.FRONT -> {
                when (activeMode) {
                    VisionHeightMode.NONE -> hatchStationFrontRegression
                    VisionHeightMode.HATCH_INTAKE -> hatchStationFrontRegression
                    VisionHeightMode.HATCH_SCORE ->
                        if (isAuto) hatchScoringAutoFrontRegression else hatchScoringFrontRegression
                    VisionHeightMode.CARGO_SCORE_ROCKET -> cargoScoringRocketFrontRegression
                }
            }

            SuperstructureRoutines.Side.BACK -> {
                when (activeMode) {
                    VisionHeightMode.NONE -> hatchStationBackRegression
                    VisionHeightMode.HATCH_INTAKE -> hatchStationBackRegression
                    VisionHeightMode.HATCH_SCORE -> hatchScoringBackRegression
                    VisionHeightMode.CARGO_SCORE_ROCKET -> cargoScoringRocketBackRegression
                }
            }
        }
    }

    /**
     * Solves the angular offset from the target
     */
    fun solve(seesTarget: Boolean, area: Double, targetAngle: Double, robotToCamera: Pose2d): Double {
        if (seesTarget) {
            val distance = activeRegression.interpolate(area)
            val targetAngleRads = -Math.toRadians(targetAngle)
            if (distance > 0.0) {
                val cameraToTarget = Pose2d.fromTranslation(Translation2d(distance, distance * tan(targetAngleRads)))
                val robotToTarget = robotToCamera.transformBy(cameraToTarget).transformBy(activeRegression.offset)
                val bearing = robotToTarget.translation.direction().rotateBy(robotToCamera.rotation).degrees
                if (bearing in -20.0..20.0) {
                    return bearing
                }
            }
        }
        return Double.NaN
    }
}