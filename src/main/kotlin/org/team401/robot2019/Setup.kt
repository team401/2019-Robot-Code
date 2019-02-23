package org.team401.robot2019

import org.snakeskin.auto.AutoManager
import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.*
import org.snakeskin.registry.Controllers
import org.snakeskin.rt.RealTimeExecutor
import org.team401.robot2019.control.superstructure.SuperstructureUpdater
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.subsystems.*
import org.team401.taxis.diffdrive.autotune.autos.TuningAutoCollectDynamicsData

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
@Setup
fun setup() {
    ControlPoller.pollInAutonomous = true
    RealTimeExecutor.rate = 0.01

    AutoManager.setAutoLoop(TuningAutoCollectDynamicsData(DrivetrainSubsystem))

    Subsystems.add(DrivetrainSubsystem, ArmSubsystem, WristSubsystem)
    Controllers.add(LeftStick, RightStick)

    RealTimeExecutor.addTask(DrivetrainSubsystem.stateEstimator)
    SuperstructureMotionPlanner.preCompile()
    RealTimeExecutor.addTask(SuperstructureUpdater)
}