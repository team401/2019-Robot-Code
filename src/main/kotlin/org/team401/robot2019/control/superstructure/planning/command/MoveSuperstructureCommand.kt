package org.team401.robot2019.control.superstructure.planning.command

import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.ArmMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

/**
 * @author Cameron Earle
 * @version 2/12/2019
 *
 */
class MoveSuperstructureCommand(val start: Point2d, val end: Point2d, val tool: WristMotionPlanner.Tool): SuperstructureCommand() {
    override fun entry() {
        //Set the waypoints for the arm motion planner.  This should also reset it
        ArmMotionPlanner.setDesiredTrajectory(start, end)
        //Tell the wrist motion planner to
        WristMotionPlanner.setToParallelMode(tool, end)
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState) {
        val armCommand = ArmMotionPlanner.update(dt) //Update the arm motion planner
        val wristCommand = WristMotionPlanner.update(armState, wristState)
        SuperstructureController.update(armCommand, wristCommand, tool)
    }

    override fun isDone(): Boolean {
        return ArmMotionPlanner.isDone() //We're done when the arm planner is finished executing the trajectory
    }
}