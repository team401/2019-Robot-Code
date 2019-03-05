package org.team401.robot2019

import org.snakeskin.auto.AutoManager
import org.snakeskin.component.ISensoredGearbox
import org.snakeskin.component.IYawSensoredDifferentialDrivetrain
import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.*
import org.snakeskin.measure.Feet
import org.snakeskin.registry.Controllers
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.utility.Selectable
import org.team401.robot2019.config.Physics
import org.team401.robot2019.control.superstructure.SuperstructureUpdater
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.subsystems.*
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoCollectDynamicsData
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoTuneTrackScrubFactor
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoTuneWheelRadius

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

    Selectable.selected = RobotIndex.PRACTICE

    AutoManager.setAutoLoop(TuningAutoCollectDynamicsData(DrivetrainSubsystem))

    //AutoManager.setAutoLoop(TuningAutoCollectDynamicsData(DrivetrainSubsystem))

    //AutoManager.setAutoLoop(TuningAutoTuneTrackScrubFactor(DrivetrainSubsystem, 10, .5))

    Subsystems.add(DrivetrainSubsystem)
    Controllers.add(LeftStick, RightStick)

    //Subsystems.add(DrivetrainSubsystem, ArmSubsystem, WristSubsystem)
    //Controllers.add(LeftStick, RightStick, Gamepad)

    RealTimeExecutor.addTask(DrivetrainSubsystem.stateEstimator)
    //SuperstructureMotionPlanner.preCompile()
    //RealTimeExecutor.addTask(SuperstructureUpdater)
}