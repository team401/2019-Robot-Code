package org.team401.robot2019.auto.steps.superstructure

import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

class SuperstructureSwitchToolStep(val tool: WristMotionPlanner.Tool): SuperstructureAutoStep() {
    override fun command(currentTime: Double) {
        SuperstructureMotionPlanner.requestToolChange(tool)
    }
}