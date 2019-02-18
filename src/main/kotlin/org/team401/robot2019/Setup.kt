package org.team401.robot2019

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Solenoid
import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.*
import org.snakeskin.measure.*
import org.snakeskin.registry.Controllers
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureUpdater
import org.team401.robot2019.subsystems.*

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
@Setup
fun setup() {
    ControlPoller.pollInAutonomous = true
    RealTimeExecutor.rate = 0.01

    Subsystems.add(WristSubsystem)
    Controllers.add(Gamepad)

    //RealTimeExecutor.addTask(SuperstructureUpdater)
    //RealTimeExecutor.addTask(DrivetrainSubsystem.stateEstimator)
}