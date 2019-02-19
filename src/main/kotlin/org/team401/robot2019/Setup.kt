package org.team401.robot2019

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.controls.ControlPoller
import org.snakeskin.dsl.*
import org.snakeskin.measure.*
import org.snakeskin.registry.Controllers
import org.snakeskin.rt.RealTimeExecutor
import org.snakeskin.rt.RealTimeTask
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureUpdater
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.*
import org.team401.robot2019.subsystems.arm.control.ArmKinematics

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
@Setup
fun setup() {
    ControlPoller.pollInAutonomous = true
    RealTimeExecutor.rate = 0.01

    Subsystems.add(ArmSubsystem, WristSubsystem)
    Controllers.add(Gamepad)

    //RealTimeExecutor.addTask(SuperstructureUpdater)

    SmartDashboard.putNumber("ArmKv", 0.0)
    //RealTimeExecutor.addTask(DrivetrainSubsystem.stateEstimator)

    //What is about to happen is very dumb
    SuperstructureMotionPlanner.startUp(
        ArmState(
            Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
            0.0.Radians,
            0.0.RadiansPerSecond
        ),
        WristState(0.0.Radians, false, false),
        WristMotionPlanner.Tool.HatchPanelTool
    )

    /*
    SuperstructureMotionPlanner.requestMove(ArmKinematics.forward(
        PointPolar(
            Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
            Math.PI.Radians
    )))
    var time = 0.0
    val dt = 0.1

    for (i in 0 until 1000) {
        SuperstructureMotionPlanner.update(time, dt,  ArmState(
            Geometry.ArmGeometry.minSafeArmLength + 1.0.Inches,
            0.0.Radians,
            0.0.RadiansPerSecond
        ), WristState(0.0.Radians, false, false))
        time += dt
    }

    SuperstructureMotionPlanner.startUp(ArmState(0.0.Inches, 0.0.Radians, 0.0.RadiansPerSecond),
        WristState(0.0.Radians, false, false), WristMotionPlanner.Tool.HatchPanelTool)

    RealTimeExecutor.addTask(SuperstructureUpdater)
    */
}