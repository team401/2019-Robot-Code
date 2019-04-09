package org.team401.robot2019

import org.snakeskin.auto.AutoManager
import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.Setup
import org.snakeskin.dsl.Subsystems
import org.snakeskin.dsl.on
import org.snakeskin.event.Events
import org.snakeskin.registry.Controllers
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.utility.Selectable
import org.team401.robot2019.auto.DeepSpaceAuto
import org.team401.robot2019.control.drivetrain.CriticalPoses
import org.team401.robot2019.control.drivetrain.Trajectories
import org.team401.robot2019.control.superstructure.SuperstructureUpdater
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.control.vision.VisionOdometryUpdater
import org.team401.robot2019.subsystems.ArmSubsystem
import org.team401.robot2019.subsystems.ClimberSubsystem
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.robot2019.subsystems.WristSubsystem
import org.team401.robot2019.util.LEDManager
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.TrajectoryIterator

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
object RobotIndex {
    const val COMP = 0
    const val PRACTICE = 1
}


@Setup
fun setup() {
    ControlPoller.pollInAutonomous = true
    RealTimeExecutor.rate = 0.01

    Selectable.selected = RobotIndex.PRACTICE //CHANGE THIS depending on what robot you're using.

    AutoManager.setAutoLoop(DeepSpaceAuto)

    //AutoManager.setAutoLoop(TuningAutoCollectDynamicsData(DrivetrainSubsystem))

    //Register components
    Subsystems.add(DrivetrainSubsystem, ArmSubsystem, WristSubsystem, ClimberSubsystem)
    Controllers.add(LeftStick, RightStick, Gamepad)

    //Miscellaneous initialization
    LEDManager.init()
    VisionManager.stop()
    DriverStationDisplay.init()
    SuperstructureMotionPlanner.preCompile()

    //Initialize real-time tasks
    RealTimeExecutor.addTask(DrivetrainSubsystem.stateEstimator) //Drivetrain odometry from sensors
    RealTimeExecutor.addTask(VisionOdometryUpdater)              //Drivetrain odometry from vision
    RealTimeExecutor.addTask(SuperstructureUpdater)              //Superstrcture motion planning / control
    //RealTimeExecutor.addTask(OdometryWatchdog)                   //Drivetrain odometry error checking

    on (Events.TELEOP_ENABLED) {
        VisionManager.frontCamera.configForVision(3)
        VisionManager.backCamera.configForVision(3)
        VisionManager.frontCamera.setLedMode(LimelightCamera.LedMode.Off)
        VisionManager.backCamera.setLedMode(LimelightCamera.LedMode.Off)
        VisionManager.stop()
    }

    val trajectory = Trajectories.level1HabToNearRocketLeft
    val endPose = trajectory.lastState.state().pose
    val endPoseReal = CriticalPoses.fieldToNearRocketLeft//.transformBy(CriticalPoses.robotFrontCenterToOriginTransform)
    println("Trajectory end pose: $endPose")
    println("Real end pose: $endPoseReal")
}