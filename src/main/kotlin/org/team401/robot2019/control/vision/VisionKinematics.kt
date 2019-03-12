package org.team401.robot2019.control.vision

import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d

object VisionKinematics {
    /**
     * Constant transform from the robot to the camera.
     * Distance from the origin of the robot to the origin of the camera, as measured in the field (x, y) configuration
     */
    val robotToCamera = Pose2d(0.0, 0.0, Rotation2d.identity())

    /**
     * @param fieldToRobot Pose measurement of the robot relative to the field origin frame
     * @param fieldToGoal Pose measurement of the goal relative to the field origin frame
     * @param cameraToGoal Pose measurement of the goal relative to the origin of the camera
     *
     * @return The actual field to robot measurement from the camera
     */
    fun forward(fieldToRobot: Pose2d, fieldToGoal: Pose2d, cameraToGoal: Pose2d): Pose2d {
        //Calculate the pose of the camera, from odometry, on the field
        val fieldToCamera = fieldToRobot.transformBy(robotToCamera)
        //Calculate the field to goal transform, as measured by the camera
        val fieldToGoalMeasured = fieldToCamera.transformBy(cameraToGoal.inverse())
        //Calculate the difference between the expected field to goal and the actual field to goal
        val error = fieldToGoal.transformBy(fieldToGoalMeasured.inverse())
        //Transform the original robot pose by this displacement to arrive at actual robot pose
        return fieldToRobot.transformBy(error)
    }
}

fun main(args: Array<String>) {
    println(VisionKinematics.forward(
        Pose2d(20.0, -10.0, Rotation2d.fromDegrees(90.0)),
        Pose2d(20.0, 0.0, Rotation2d.fromDegrees(90.0)),
        Pose2d(-10.0, 0.0, Rotation2d.fromDegrees(0.0))
    ))
}