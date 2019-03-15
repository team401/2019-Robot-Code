package org.team401.robot2019

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.auto.AutoManager
import org.snakeskin.component.ISensoredGearbox
import org.snakeskin.component.IYawSensoredDifferentialDrivetrain
import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.*
import org.snakeskin.factory.ExecutorFactory
import org.snakeskin.measure.Feet
import org.snakeskin.measure.Seconds
import org.snakeskin.registry.Controllers
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.utility.Selectable
import org.team401.robot2019.auto.CollectAngularTorqueData
import org.team401.robot2019.auto.CollectLinearTorqueData
import org.team401.robot2019.config.HardwareMap
import org.team401.robot2019.config.Physics
import org.team401.robot2019.control.superstructure.SuperstructureUpdater
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.vision.VisionKinematics
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.subsystems.*
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoCollectDynamicsData
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoTuneTrackScrubFactor
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoTuneWheelRadius
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.trajectory.TimedView
import org.team401.taxis.trajectory.TrajectoryIterator
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

    Selectable.selected = RobotIndex.COMP

    //AutoManager.setAutoLoop(CollectLinearTorqueData.createAuto(DrivetrainSubsystem, .5, 2.0.Seconds, 1, -1.0))

    AutoManager.setAutoLoop(CollectAngularTorqueData.createAuto(DrivetrainSubsystem, 0.5, 2.0.Seconds, 16.49251069,2))

    //AutoManager.setAutoLoop(TuningAutoTuneTrackScrubFactor(DrivetrainSubsystem, 10, .25))

    //AutoManager.setAutoLoop(TuningAutoTuneTrackScrubFactor(DrivetrainSubsystem, 10, .5))

    //AutoManager.setAutoLoop(TuningAutoTuneTrackScrubFactor(DrivetrainSubsystem, 10, 0.25))

    //Subsystems.add(DrivetrainSubsystem, ArmSubsystem, WristSubsystem, FloorPickupSubsystem)
    Subsystems.add(DrivetrainSubsystem, ClimberSubsystem)
    Controllers.add(LeftStick, RightStick)

    //VisionManager.start()

    /*
    ExecutorFactory.getExecutor("test").scheduleAtFixedRate({
        val cameraToGoal = VisionKinematics.cameraForward(
            DrivetrainSubsystem.getHeading(),
            Rotation2d.fromDegrees(0.0),
            Rotation2d.fromDegrees(VisionManager.backCamera.frame.targetAngleHoriz.value),
            Rotation2d.fromDegrees(VisionManager.backCamera.frame.targetAngleVert.value)
        )
        println(cameraToGoal)
    }, 0L, 100L, TimeUnit.MILLISECONDS)
    */

    //Subsystems.add(DrivetrainSubsystem, ArmSubsystem, WristSubsystem)
    //Controllers.add(LeftStick, RightStick, Gamepad)

    //RealTimeExecutor.addTask(DrivetrainSubsystem.stateEstimator)
    //SuperstructureMotionPlanner.preCompile()
    //RealTimeExecutor.addTask(SuperstructureUpdater)
}