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
import org.team401.robot2019.control.drivetrain.Trajectories
import org.team401.robot2019.control.superstructure.SuperstructureUpdater
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.subsystems.*
import org.team401.robot2019.util.LEDManager

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

    Selectable.selected = RobotIndex.COMP //DO NOT CHANGE THIS

    AutoManager.setAutoLoop(DeepSpaceAuto)
    DeepSpaceAuto.publish()

    //AutoManager.setAutoLoop(TuningAutoCollectDynamicsData(DrivetrainSubsystem))

    //Register components
    Subsystems.add(DrivetrainSubsystem, ArmSubsystem, WristSubsystem, FloorPickupSubsystem, ClimberSubsystem)
    Controllers.add(LeftStick, RightStick, Gamepad)

    //Miscellaneous initialization
    LEDManager.init()
    VisionManager.stop()
    DriverStationDisplay.init()
    SuperstructureMotionPlanner.preCompile()

    //Initialize real-time tasks
    RealTimeExecutor.addTask(DrivetrainSubsystem.stateEstimator) //Drivetrain odometry from sensors
    RealTimeExecutor.addTask(SuperstructureUpdater)              //Superstrcture motion planning / control

    on (Events.TELEOP_ENABLED) {
        VisionManager.frontCamera.configForVision(1)
        VisionManager.backCamera.configForVision(3)
        VisionManager.frontCamera.setLedMode(LimelightCamera.LedMode.Off)
        VisionManager.backCamera.setLedMode(LimelightCamera.LedMode.Off)
        VisionManager.stop()
    }

    Trajectories //Load this class so the trajectories get pregenerated
}