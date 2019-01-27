package org.team401.armsim

import org.team401.armsim.profile.ArmPath
import org.team401.armsim.profile.LinearProfileSegment
import org.team401.armsim.profile.Profile2d

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
fun main(args: Array<String>) {

    val desiredPath = LinearProfileSegment(Point2d(-5.0, 0.0), Point2d(5.0, -2.0))
    val path = ArmPath(desiredPath)
    val profile = Profile2d(path.solve())
    val points = profile.solveSystem()

    val xSeries = DoubleArray(points.size) { points[it].first.x}
    val ySeries = DoubleArray(points.size) { points[it].first.y }
    val rSeries = DoubleArray(points.size){points[it].second.r}
    val thetaSeries = DoubleArray(points.size){points[it].second.theta}
    val timeSeries = profile.getTime()

    println("Time series length: ${timeSeries.size}")
    println("Radius series length: ${rSeries.size}")


    //val sliceSeries = DoubleArray(polar.size) { polar[it].first }
    //val rSeries = DoubleArray(points.size) { points[it].second.r }
    //val thetaSeries = DoubleArray(points.size) { Math.toDegrees(points[it].second.theta) }

    /*
    val xyChart = QuickChart.getChart("XY Pose", "x", "y", "y(x)", xSeries, ySeries)
    val rChart = QuickChart.getChart("Arm Radius vs Time", "Time", "Radius", "r(t)", timeSeries, rSeries)
    val thetaChart = QuickChart.getChart("Arm Angle vs Time", "Time", "Theta", "theta(t)", timeSeries, thetaSeries)
    SwingWrapper(xyChart).displayChart()
    SwingWrapper(rChart).displayChart()
    SwingWrapper(thetaChart).displayChart()
    */
}