package org.team401.robot2019.control.vision

import org.team401.taxis.geometry.Pose2d

object VisionKinematics {
    /**
     * Solves the actual field to robot pose given the estimated robot pose, the constant field to goal,
     * the constant robot to camera, and the measured camera to goal
     *
     * @param fieldToRobot Pose measurement of the robot relative to the field origin frame
     * @param fieldToGoal Pose measurement of the goal relative to the field origin frame
     * @param robotToCamera Offset from the origin of the robot to the camera
     * @param cameraToGoal Pose measurement of the goal relative to the origin of the camera
     *
     * @return The actual field to robot measurement from the camera
     */
    fun solveFieldToRobot(fieldToRobot: Pose2d, fieldToGoal: Pose2d, robotToCamera: Pose2d, cameraToGoal: Pose2d): Pose2d {
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
     * Solves the field to goal pose given the current robot pose, and the camera to goal pose
     *
     * @param fieldToRobot Pose measurement of the robot relative to the field origin frame
     * @param robotToCamera Offset from the origin of the robot to the camera
     * @param cameraToGoal Pose measurement of the goal relative to the origin of the camera
     */
    fun solveFieldToGoal(fieldToRobot: Pose2d, robotToCamera: Pose2d, cameraToGoal: Pose2d): Pose2d {
        //Calculate the pose of the camera, from odometry, on the field
        val fieldToCamera = fieldToRobot.transformBy(robotToCamera)
        //Transform current odometry position by camera readings to get pose of target
        return fieldToCamera.transformBy(cameraToGoal.inverse())
    }
}