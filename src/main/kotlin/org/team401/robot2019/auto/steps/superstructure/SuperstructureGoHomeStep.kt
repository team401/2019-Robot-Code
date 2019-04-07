package org.team401.robot2019.auto.steps.superstructure

import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner

class SuperstructureGoHomeStep: SuperstructureAutoStep() {
    override fun command(currentTime: Double) {
        SuperstructureMotionPlanner.goHome()
    }
}