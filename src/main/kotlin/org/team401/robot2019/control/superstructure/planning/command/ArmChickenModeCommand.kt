package org.team401.robot2019.control.superstructure.planning.command

import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.VisionHeightMode
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.ArmMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.taxis.geometry.Pose2d

class ArmChickenModeCommand(val tool: WristMotionPlanner.Tool): SuperstructureCommand() {
    var firstRun = true

    override fun entry() {
    }

    override fun action(dt: Double, armState: ArmState, wristState: WristState, drivePose: Pose2d) {
        if (firstRun) {
            ArmMotionPlanner.setChickenModeOn(drivePose)
            firstRun = false
        }
        val armCommand = ArmMotionPlanner.update(dt, drivePose) //Update the arm motion planner
        WristMotionPlanner.setToParallelMode(tool, ArmKinematics.forward(armCommand))
        val wristCommand = WristMotionPlanner.update(armState, wristState) //Update the wrist motion planner
        SuperstructureController.update(armCommand, wristCommand, tool, VisionHeightMode.NONE)
    }

    override fun isDone(): Boolean {
        return false//ArmMotionPlanner.isDone() //We're done when the arm planner is finished executing the trajectory
    }

    override fun getDescription(): String {
        return "CHICKEN"
    }
}