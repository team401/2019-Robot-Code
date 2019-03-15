package org.team401.robot2019.control.vision

import org.snakeskin.measure.Inches
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

object VisionKinematics {
    /**
     * Constant transform from the robot to the camera.
     * Distance from the origin of the robot to the origin of the camera, as measured in the field (x, y) configuration
     */
    val robotToCamera = Pose2d(0.0, 0.0, Rotation2d.identity())

    val cameraToGoalHeight = 12.125.Inches

    /**
     * @param fieldToRobot Pose measurement of the robot relative to the field origin frame
     * @param fieldToGoal Pose measurement of the goal relative to the field origin frame
     * @param cameraToGoal Pose measurement of the goal relative to the origin of the camera
     *
     * @return The actual field to robot measurement from the camera
     */
    fun fieldForward(fieldToRobot: Pose2d, fieldToGoal: Pose2d, cameraToGoal: Pose2d): Pose2d {
        //Calculate the pose of the camera, from odometry, on the field
        val fieldToCamera = fieldToRobot.transformBy(robotToCamera)
        //Calculate the field to goal transform, as measured by the camera
        val fieldToGoalMeasured = fieldToCamera.transformBy(cameraToGoal.inverse())
        //Calculate the difference between the expected field to goal and the actual field to goal
        val error = fieldToGoal.transformBy(fieldToGoalMeasured.inverse())
        //Transform the original robot pose by this displacement to arrive at actual robot pose
        return fieldToRobot.transformBy(error)
    }

    /**
     * Solves the camera to goal pose given the angle of the robot, the angle of the target, and the camera info
     */
    fun cameraForward(robotAngle: Rotation2d, goalAngle: Rotation2d, cameraHorizontal: Rotation2d, cameraVertical: Rotation2d): Pose2d {
        val angleFromGoal = goalAngle.rotateBy(robotAngle.inverse())
        //println("angleFromGoal: $angleFromGoal")
        val cameraHorizontalAdjusted = cameraHorizontal.rotateBy(angleFromGoal)
        //println("cameraHorizontalAdjusted: $cameraHorizontalAdjusted")
        val cameraDistanceToGoal = cameraToGoalHeight.value / cameraVertical.tan()
        //println("dist: $cameraDistanceToGoal")
        val cameraHorizontalDisplacementToGoal = cameraDistanceToGoal * cameraHorizontalAdjusted.tan()
        //println("disp: $cameraHorizontalDisplacementToGoal")
        return Pose2d(-cameraDistanceToGoal, -cameraHorizontalDisplacementToGoal, angleFromGoal)
    }
}

fun main(args: Array<String>) {
    println(Pose2d(20.0, 20.0, Rotation2d.identity()).inverse().transformBy(Pose2d(22.0, 22.0, Rotation2d.identity())))
}