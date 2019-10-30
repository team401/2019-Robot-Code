package org.team401.robot2019.auto.steps.drivetrain

import org.snakeskin.auto.steps.AutoStep
import org.team401.robot2019.auto.steps.InterruptableAutoStep
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.taxis.geometry.Pose2dWithCurvature
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.Trajectory
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.trajectory.timing.TimedState

/**
 * @author Cameron Earle
 * @version 3/25/2019
 *
 * Auto step that drives a trajectory.  Can optionally reconfigure the pose of the system before starting.
 */
class DriveTrajectoryStep(referenceTrajectory: Trajectory<TimedState<Pose2dWithCurvature>>, val reconfigurePose: Boolean = false): AutoStep() {
    private val trajectory = TrajectoryIterator(TimedView(referenceTrajectory))

    override fun entry(currentTime: Double) {
        if (reconfigurePose) { //If we're supposed to reconfigure the pose
            val startPose = trajectory.state.state().pose //Grab the initial pose
            DrivetrainSubsystem.setPose(startPose, currentTime) //Reconfigure the drive
        }
        DrivetrainSubsystem.pathManager.reset() //Reset the path manager
        DrivetrainSubsystem.pathManager.setTrajectory(trajectory) //Load in the path
        DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.PathFollowing).waitFor() //Begin following
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        return DrivetrainSubsystem.pathManager.isDone //Wait for the path manager to be done
    }

    override fun exit(currentTime: Double) {
        DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.Disabled) //Stop the drive
    }
}