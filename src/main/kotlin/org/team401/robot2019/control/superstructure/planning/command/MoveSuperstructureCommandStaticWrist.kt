package org.team401.robot2019.control.superstructure.planning.command

import org.snakeskin.measure.Degrees
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
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
class MoveSuperstructureCommandStaticWrist(val start: Point2d, val end: Point2d, val tool: WristMotionPlanner.Tool, val wristAngle: AngularDistanceMeasureRadians, val minimumRadius: LinearDistanceMeasureInches): SuperstructureCommand() {
    override fun entry() {
        println("STARTING MOVE: $start to $end")
        //Set the waypoints for the arm motion planner.  This should also reset it
        ArmMotionPlanner.setDesiredTrajectory(start, end, minimumRadius)
        WristMotionPlanner.setToAngleMode(tool, wristAngle, end)
        //WristMotionPlanner.setToMaintainAngleMode(90.0.Degrees.toRadians(), tool, end)
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState) {
        val armCommand = ArmMotionPlanner.update(dt) //Update the arm motion planner
        val wristCommand = WristMotionPlanner.update(armState, wristState) //Update the wrist motion planner
        SuperstructureController.update(armCommand, wristCommand, tool)
    }

    override fun isDone(): Boolean {
        if(ArmMotionPlanner.isDone()){
            println("Move finished")
        }
        return ArmMotionPlanner.isDone() //We're done when the arm planner is finished executing the trajectory
    }
}