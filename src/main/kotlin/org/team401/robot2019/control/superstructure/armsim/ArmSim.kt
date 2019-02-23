package org.team401.robot2019.control.superstructure.armsim

import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureControlOutput
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.planning.ArmMotionPlanner
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.control.superstructure.geometry.WristState
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.control.superstructure.planning.command.MoveSuperstructureCommand
import org.team401.robot2019.control.superstructure.planning.command.SuperstructureCommand
import javax.swing.JFrame
import javax.swing.SwingUtilities
import kotlin.math.PI

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
object ArmSim {
    @JvmStatic
    fun main(args: Array<String>) {

        val points = ArrayList<PointPolar>()
        val time = ArrayList<Double>()
        var currentTime = 0.0
        val dt = 0.01
        //val startPoint = ArmKinematics.inverse(Point2d(34.55772987804611.Inches, 0.2650596661423766.Inches))
        val startPoint = ArmKinematics.inverse(Point2d((20.0).Inches, 0.0.Inches))
        val startArmState = ArmState(startPoint.r, startPoint.theta, 0.0.RadiansPerSecond)
        val startWristState = WristState(PI.Radians, false, true)
        val ffVoltage = ArrayList<Double>()
        val commands = hashMapOf<Double, SuperstructureControlOutput>()

        SuperstructureMotionPlanner.startUp(startArmState, startWristState)// TODO In real life, populate this function!!
        //SuperstructureMotionPlanner.requestMove(Point2d((0.0).Inches, 35.0.Inches))
        SuperstructureMotionPlanner.testRotationOnly((Math.PI).Radians)
        /*
        SuperstructureMotionPlanner.commandQueue.clear()
        SuperstructureMotionPlanner.commandQueue.add(MoveSuperstructureCommand(
            Point2d(14.527886757347916.Inches, 22.406082488310304.Inches),
            Point2d(18.2796944981577.Inches, 28.192423965562504.Inches),
            WristMotionPlanner.Tool.HatchPanelTool)
        )
        */
        SuperstructureMotionPlanner.update(currentTime, dt, startArmState, startWristState)

        while (!SuperstructureMotionPlanner.isDone()) {

            val output = SuperstructureController.output
            commands[currentTime] = output
            currentTime += dt
            //println("Radius: ${output.armRadius}, Angle: ${output.armAngle}")

            points.add(PointPolar(output.armRadius, output.armAngle))
            time.add(currentTime)

            val currentArmState = ArmState(output.armRadius, output.armAngle, output.armVelocity)
            val currentWristState = WristState(output.wristTheta, false, false)

            SuperstructureMotionPlanner.update(currentTime, dt, currentArmState, currentWristState)
            ffVoltage.add(SuperstructureController.output.armFeedForwardVoltage)
        }

        val xSeries = DoubleArray(points.size) { ArmKinematics.forward(points[it]).x.value }
        val ySeries = DoubleArray(points.size) { ArmKinematics.forward(points[it]).y.value }
        val rSeries = DoubleArray(points.size) { points[it].r.value }
        val thetaSeries = DoubleArray(points.size) { points[it].theta.value }
        val timeSeries = DoubleArray(time.size) { time[it]}
        val voltageSeries = DoubleArray(ffVoltage.size) { ffVoltage[it] }

        //println("Time series length: ${timeSeries.size}")
        //println("Radius series length: ${rSeries.size}")


        //val sliceSeries = DoubleArray(polar.size) { polar[it].first }
        //val rSeries = DoubleArray(points.size) { points[it].second.r }
        //val thetaSeries = DoubleArray(points.size) { Math.toDegrees(points[it].second.theta) }


        val xyChart = QuickChart.getChart("XY Pose", "x", "y", "y(x)", xSeries, ySeries)
        val rChart = QuickChart.getChart("ArmSubsystem Radius vs Time", "Time", "Radius", "r(t)", timeSeries, rSeries)
        val thetaChart = QuickChart.getChart("ArmSubsystem Angle vs Time", "Time", "Theta", "theta(t)", timeSeries, thetaSeries)
        val ffChart = QuickChart.getChart("Feedforward Voltages", "Time", "Voltage", "v(t)", timeSeries, voltageSeries)
        SwingWrapper(xyChart).displayChart()
        SwingWrapper(rChart).displayChart()
        SwingWrapper(thetaChart).displayChart()
        SwingWrapper(ffChart).displayChart()

        val frame = JFrame("Graphics")
        val canvas = SuperstructureCanvas(3.0)
        canvas.setSize(400, 400)
        frame.add(canvas)
        frame.pack()
        SwingUtilities.invokeLater {
            frame.isVisible = true
        }

        commands.forEach {
            SwingUtilities.invokeLater {
                canvas.update(
                    ArmState(
                        it.value.armRadius,
                        it.value.armAngle,
                        it.value.armVelocity
                    ),
                    WristState(
                        it.value.wristTheta,
                        false, false
                    )
                )
                println("repaint")

                canvas.repaint()
            }
        }
    }
}