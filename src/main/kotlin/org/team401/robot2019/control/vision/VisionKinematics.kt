package org.team401.robot2019.control.vision

import org.team401.taxis.geometry.Pose2d

object VisionKinematics {
    /**
     * Solves the actual field to robot pose given the estimated robot pose, the constant field to goal,
     * the constant robot to camera, and the measured camera to goal
     *
     * @param fieldToGoal Pose measurement of the goal relative to the field origin frame
     * @param robotToCamera Offset from the origin of the robot to the camera
     * @param goalToCamera Pose measurement of the goal relative to the origin of the camera
     *
     * @return The actual field to robot measurement from the camera
     */
    fun solveFieldToRobot(fieldToGoal: Pose2d, robotToCamera: Pose2d, goalToCamera: Pose2d): Pose2d {
        //Transform field to goal by the cameraToGoal to get the fieldToCamera
        val fieldToCamera = fieldToGoal.transformBy(goalToCamera)

        return fieldToCamera.transformBy(robotToCamera.inverse())
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

    fun solveLatencyCorrection(poseAtCapture: Pose2d, currentPose: Pose2d, poseToAdjust: Pose2d): Pose2d {
        val displacement = poseAtCapture.inverse().transformBy(currentPose)
        return poseToAdjust.transformBy(displacement)
    }
}