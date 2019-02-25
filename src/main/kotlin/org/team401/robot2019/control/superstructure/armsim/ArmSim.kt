package org.team401.robot2019.control.superstructure.armsim

import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.snakeskin.measure.Degrees
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureControlOutput
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.geometry.*
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import javax.swing.JFrame
import javax.swing.SwingUtilities
import kotlin.math.PI

/**
 * @author Eli Jelesko and Cameron Earle
 * @version 1/19/2019
 *
 */
object ArmSim {
    data class SimFrame(val time: Double,
                        val command: SuperstructureControlOutput)

    fun runSimulation(startPose: Point2d,
                      goal: ArmSetpoint,
                      startWrist: WristState = WristState(0.0.Radians, false, false),
                      startTool: WristMotionPlanner.Tool = WristMotionPlanner.Tool.CargoTool): List<SimFrame> {
        val frames = arrayListOf<SimFrame>()

        var currentTime = 0.0
        val dt = 0.01

        val startPoint = ArmKinematics.inverse(startPose)
        val startArmState = ArmState(startPoint.r, startPoint.theta, 0.0.RadiansPerSecond)

        SuperstructureMotionPlanner.startUp(startArmState, startWrist, startTool)
        SuperstructureMotionPlanner.requestMove(goal)

        SuperstructureMotionPlanner.update(currentTime, dt, startArmState, startWrist)

        while (!SuperstructureMotionPlanner.isDone()) {
            val output = SuperstructureController.output
            frames.add(SimFrame(currentTime, output))

            currentTime += dt
            //println("Radius: ${output.armRadius}, Angle: ${output.armAngle}")

            val currentArmState = ArmState(output.armRadius, output.armAngle, output.armVelocity)
            val currentWristState = WristState(output.wristTheta, false, false)

            SuperstructureMotionPlanner.update(currentTime, dt, currentArmState, currentWristState)
        }

        return frames
    }

    fun graphData(frames: List<SimFrame>) {
        val time = frames.map { it.time }
        val points = frames.map { PointPolar(it.command.armRadius, it.command.armAngle) }
        val ffVoltage = frames.map { it.command.armFeedForwardVoltage }

        val xSeries = DoubleArray(points.size) { ArmKinematics.forward(points[it]).x.value }
        val ySeries = DoubleArray(points.size) { ArmKinematics.forward(points[it]).y.value }
        val rSeries = DoubleArray(points.size) { points[it].r.value }
        val thetaSeries = DoubleArray(points.size) { points[it].theta.value }
        val timeSeries = DoubleArray(time.size) { time[it]}
        val voltageSeries = DoubleArray(ffVoltage.size) { ffVoltage[it] }

        val xyChart = QuickChart.getChart("XY Pose", "x", "y", "y(x)", xSeries, ySeries)
        val rChart = QuickChart.getChart("ArmSubsystem Radius vs Time", "Time", "Radius", "r(t)", timeSeries, rSeries)
        val thetaChart = QuickChart.getChart("ArmSubsystem Angle vs Time", "Time", "Theta", "theta(t)", timeSeries, thetaSeries)
        val ffChart = QuickChart.getChart("Feedforward Voltages", "Time", "Voltage", "v(t)", timeSeries, voltageSeries)
        SwingWrapper(xyChart).displayChart()
        SwingWrapper(rChart).displayChart()
        SwingWrapper(thetaChart).displayChart()
        SwingWrapper(ffChart).displayChart()
    }

    fun createSimulationGraphics(ppi: Double, data: List<SimFrame>, cargoToolLength: LinearDistanceMeasureInches, hatchToolLength: LinearDistanceMeasureInches) {
        val frame = SuperstructureGraphicsFrame(ppi, 0.01, 15.0, data, cargoToolLength, hatchToolLength)
        SwingUtilities.invokeLater {
            frame.pack()
            frame.isVisible = true
        }
    }

    @JvmStatic
    fun main(args: Array<String>) {
        val output = runSimulation(Point2d((0.0).Inches, Geometry.ArmGeometry.maxY), ArmSetpoint(
            Point2d((40.0).Inches, 0.0.Inches), WristMotionPlanner.Tool.CargoTool, 0.0.Radians
        )
        )


        graphData(output)
        createSimulationGraphics(3.0, output, 14.0.Inches, 10.0.Inches)
    }
}