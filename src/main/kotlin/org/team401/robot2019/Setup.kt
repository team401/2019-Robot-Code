package org.team401.robot2019

import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.*
import org.snakeskin.measure.*
import org.snakeskin.registry.Controllers
import org.snakeskin.rt.RealTimeExecutor
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.subsystems.arm.Arm
import org.team401.robot2019.subsystems.drivetrain.Drivetrain

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
@Setup
fun setup() {
    ControlPoller.pollInAutonomous = true
    RealTimeExecutor.rate = 0.01

    Subsystems.add(Drivetrain)
    Controllers.add(LeftStick, RightStick)

    RealTimeExecutor.addTask(Drivetrain.stateEstimator)
}

fun main(args: Array<String>) {
    println(0.5.FeetPerSecond.toAngularVelocity(Geometry.DrivetrainGeometry.wheelRadius).toRadiansPerSecond())
}