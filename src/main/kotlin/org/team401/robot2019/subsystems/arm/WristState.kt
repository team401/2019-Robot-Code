package org.team401.robot2019.subsystems.arm

import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians


data class WristState(val wristPosition: AngularDistanceMeasureRadians, val currentTool: Tool, val hasGamePiece: Boolean)

enum class Tool{
    CARGO_INTAKE, HATCH_INTAKE
}