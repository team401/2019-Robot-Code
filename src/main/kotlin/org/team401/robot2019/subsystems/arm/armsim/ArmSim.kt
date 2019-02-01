package org.team401.armsim

import org.knowm.xchart.QuickChart
import org.knowm.xchart.SwingWrapper
import org.snakeskin.units.Inches
import org.snakeskin.units.measure.time.TimeMeasureSeconds
import org.team401.robot2019.subsystems.arm.ArmPather
import org.team401.robot2019.subsystems.arm.ArmState

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
object ArmSim {
    @JvmStatic
    fun main(args: Array<String>) {

        ArmPather.reset()
        ArmPather.setDesiredPath(Point2d((-5.0).Inches, 0.0.Inches), Point2d(5.0.Inches, (-2.0).Inches))

        val points = ArrayList<ArmState>()
        val time = ArrayList<TimeMeasureSeconds>()

        while (!ArmPather.isDone()) {
            points.add(ArmPather.update())
            time.add(ArmPather.getCurrentTime())
        }

        val xSeries = DoubleArray(points.size) { ArmKinematics.forward(points[it]).x.value }
        val ySeries = DoubleArray(points.size) { ArmKinematics.forward(points[it]).y.value }
        val rSeries = DoubleArray(points.size) { points[it].position.first.value }
        val thetaSeries = DoubleArray(points.size) { points[it].position.second.value }
        val velocitySeries = DoubleArray(points.size){points[it].armVelocity.value}
        val timeSeries = DoubleArray(time.size) { time[it].value }

        //println("Time series length: ${timeSeries.size}")
        //println("Radius series length: ${rSeries.size}")


        //val sliceSeries = DoubleArray(polar.size) { polar[it].first }
        //val rSeries = DoubleArray(points.size) { points[it].second.r }
        //val thetaSeries = DoubleArray(points.size) { Math.toDegrees(points[it].second.theta) }


        val xyChart = QuickChart.getChart("XY Pose", "x", "y", "y(x)", xSeries, ySeries)
        val rChart = QuickChart.getChart("Arm Radius vs Time", "Time", "Radius", "r(t)", timeSeries, rSeries)
        val thetaChart = QuickChart.getChart("Arm Angle vs Time", "Time", "Theta", "theta(t)", timeSeries, thetaSeries)
        val velocityChart = QuickChart.getChart("Velocity vs Time", "Time", "Velocity", "v(t)", timeSeries, velocitySeries)
        SwingWrapper(xyChart).displayChart()
        SwingWrapper(rChart).displayChart()
        SwingWrapper(thetaChart).displayChart()
        SwingWrapper(velocityChart).displayChart()

    }
}