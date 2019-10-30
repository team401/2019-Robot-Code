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
    private val hatchStationFrontRegression = VisionRegression(81.892342, -0.493349)
    private val hatchStationBackRegression = hatchStationFrontRegression
    private val hatchScoringFrontRegression = VisionRegression(81.892342, -0.493349)
    private val hatchScoringBackRegression = hatchScoringFrontRegression
    private val cargoScoringRocketFrontRegression = VisionRegression(81.892342, -0.493349)
    private val cargoScoringRocketBackRegression = cargoScoringRocketFrontRegression

    private var activeRegression = hatchStationFrontRegression

    /**
     * Selects the correct regression to use given the conditions
     */
    fun selectRegression(activeSide: SuperstructureRoutines.Side, activeMode: VisionHeightMode) {
        activeRegression = when (activeSide) {
            SuperstructureRoutines.Side.FRONT -> {
                when (activeMode) {
                    VisionHeightMode.NONE -> hatchStationFrontRegression
                    VisionHeightMode.HATCH_INTAKE -> hatchStationFrontRegression
                    VisionHeightMode.HATCH_SCORE -> hatchScoringFrontRegression
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
                val robotToTarget = robotToCamera.transformBy(cameraToTarget)
                val bearing = robotToTarget.translation.direction().rotateBy(robotToCamera.rotation).degrees
                if (bearing in -20.0..20.0) {
                    return bearing
                }
            }
        }
        return Double.NaN
    }
}