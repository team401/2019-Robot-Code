package org.team401.robot2019.control.superstructure.armsim

import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.snakeskin.measure.Inches
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2019.control.superstructure.planning.ArmMotionPlanner
import org.team401.robot2019.control.superstructure.geometry.ArmState
import org.team401.robot2019.subsystems.arm.control.ArmKinematics
import org.team401.robot2019.control.superstructure.geometry.Point2d

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
object ArmSim {
    @JvmStatic
    fun main(args: Array<String>) {

        ArmMotionPlanner.reset()
        ArmMotionPlanner.setDesiredTrajectory(Point2d((0.0).Inches, 12.0.Inches), Point2d((0.0).Inches, (20.0).Inches))

        val points = ArrayList<ArmState>()
        val time = ArrayList<TimeMeasureSeconds>()

        while (!ArmMotionPlanner.isDone()) {
            points.add(ArmMotionPlanner.update(0.01))
            time.add(ArmMotionPlanner.getCurrentTime())
        }

        val xSeries = DoubleArray(points.size) { ArmKinematics.forward(points[it]).x.value }
        val ySeries = DoubleArray(points.size) { ArmKinematics.forward(points[it]).y.value }
        val rSeries = DoubleArray(points.size) { points[it].armRadius.value }
        val thetaSeries = DoubleArray(points.size) { points[it].armAngle.value }
        val velocitySeries = DoubleArray(points.size){points[it].armVelocity.value}
        val timeSeries = DoubleArray(time.size) { time[it].value }

        //println("Time series length: ${timeSeries.size}")
        //println("Radius series length: ${rSeries.size}")


        //val sliceSeries = DoubleArray(polar.size) { polar[it].first }
        //val rSeries = DoubleArray(points.size) { points[it].second.r }
        //val thetaSeries = DoubleArray(points.size) { Math.toDegrees(points[it].second.theta) }


        val xyChart = QuickChart.getChart("XY Pose", "x", "y", "y(x)", xSeries, ySeries)
        val rChart = QuickChart.getChart("ArmSubsystem Radius vs Time", "Time", "Radius", "r(t)", timeSeries, rSeries)
        val thetaChart = QuickChart.getChart("ArmSubsystem Angle vs Time", "Time", "Theta", "theta(t)", timeSeries, thetaSeries)
        val velocityChart = QuickChart.getChart("Velocity vs Time", "Time", "Velocity", "v(t)", timeSeries, velocitySeries)
        SwingWrapper(xyChart).displayChart()
        SwingWrapper(rChart).displayChart()
        SwingWrapper(thetaChart).displayChart()
        SwingWrapper(velocityChart).displayChart()

    }
}