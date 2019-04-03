package org.team401.robot2019

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.auto.AutoManager
import org.snakeskin.component.ISensoredGearbox
import org.snakeskin.component.IYawSensoredDifferentialDrivetrain
import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.*
import org.snakeskin.event.Events
import org.snakeskin.factory.ExecutorFactory
import org.snakeskin.measure.Feet
import org.snakeskin.measure.Seconds
import org.snakeskin.registry.Controllers
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.utility.Selectable
import org.team401.robot2019.auto.CollectAngularTorqueData
import org.team401.robot2019.auto.CollectLinearTorqueData
import org.team401.robot2019.auto.DeepSpaceAuto
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.config.Physics
import org.team401.robot2019.control.drivetrain.OdometryWatchdog
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.SuperstructureUpdater
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.robot2019.control.vision.VisionKinematics
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.control.vision.VisionOdometryUpdater
import org.team401.robot2019.subsystems.*
import org.team401.robot2019.util.LEDManager
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoCollectDynamicsData
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoTuneTrackScrubFactor
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoTuneWheelRadius
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.TrajectoryIterator
import org.team401.taxis.util.InterpolatingDouble
import java.util.concurrent.TimeUnit

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
    VisionManager.start()
    DriverStationDisplay.init()
    SuperstructureMotionPlanner.preCompile()

    //Initialize real-time tasks
    RealTimeExecutor.addTask(DrivetrainSubsystem.stateEstimator) //Drivetrain odometry from sensors
    RealTimeExecutor.addTask(VisionOdometryUpdater)              //Drivetrain odometry from vision
    RealTimeExecutor.addTask(SuperstructureUpdater)              //Superstrcture motion planning / control
    //RealTimeExecutor.addTask(OdometryWatchdog)                   //Drivetrain odometry error checking

    on (Events.AUTO_ENABLED) {
        VisionManager.frontCamera.configForVision(0)
        VisionManager.frontCamera.configForVision(0)
        //VisionManager.frontCamera.setStreamingMode(LimelightCamera.StreamingMode.PipMain)
    }

    on (Events.TELEOP_ENABLED) {
        VisionManager.frontCamera.configForVision(1)
        VisionManager.backCamera.configForVision(1)
        VisionManager.frontCamera.setLedMode(LimelightCamera.LedMode.Off)
        VisionManager.backCamera.setLedMode(LimelightCamera.LedMode.Off)
        //VisionManager.frontCamera.setStreamingMode(LimelightCamera.StreamingMode.PipSecondary)
    }
}