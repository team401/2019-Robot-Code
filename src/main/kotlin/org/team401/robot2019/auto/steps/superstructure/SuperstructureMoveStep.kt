package org.team401.robot2019.auto.steps.superstructure

import org.snakeskin.auto.steps.AutoStep
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.geometry.SuperstructureSetpoint
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner

class SuperstructureMoveStep(val setpoint: SuperstructureSetpoint): SuperstructureAutoStep() {
    override fun command(currentTime: Double) {
        SuperstructureMotionPlanner.requestMove(setpoint)
    }
}