package org.team401.robot2019.control.climbing

import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches

data class ClimberState(val backPosition: LinearDistanceMeasureInches,
                        val frontPosition: LinearDistanceMeasureInches)