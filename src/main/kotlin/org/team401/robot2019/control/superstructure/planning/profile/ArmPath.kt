package org.team401.robot2019.control.superstructure.planning.profile

import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.control.superstructure.geometry.CircleUtilities
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.subsystems.arm.control.ArmKinematics

class ArmPath(private val path: LinearProfileSegment, minimumRadius: LinearDistanceMeasureInches){
    // Restricts the bounds of the start and end
    private val start = path.start
    private val end = path.end

    private val x1 = path.start.x
    private val x2 = path.end.x

    private val r = minimumRadius.value

    fun solve(): Array<ProfileSegment> {
        val intersectsCircle = findIntersectionPoints()
        var segments: Array<ProfileSegment>

        if (withinCircle(start)) {
            val pointPolar = ArmKinematics.inverse(start)
            val out = PointPolar(r.Inches, pointPolar.theta)
            println("Within circle")
            return arrayOf(LinearProfileSegment(start, ArmKinematics.forward(out)))
        }
        if(withinCircle(end)){
            throw Point2d.InvalidPointException("End point $end is within minimum circle")
        }

        segments = if (intersectsCircle) {
            val tangentPointOne = findTangentPoint(start)
            val tangentPointTwo = findTangentPoint(end)

            val firstSegment = LinearProfileSegment(start, tangentPointOne)
            val secondSegment = ArcProfileSegment(tangentPointOne, tangentPointTwo, r)
            val thirdSegment = LinearProfileSegment(tangentPointTwo, end)

            arrayOf(firstSegment, secondSegment, thirdSegment)

        }else{
            arrayOf(LinearProfileSegment(start, end))
        }
        //segments.forEach { println("${it.start}, ${it.end}") }

        return segments
    }

    private fun findTangentPoint(point: Point2d): Point2d {
        val tangentPoint: Point2d
        val solutions = CircleUtilities.identifyTangentPoints(r.Inches, point)

        val startTheta = ArmKinematics.inverse(point).theta.value
        val endTheta = ArmKinematics.inverse(end).theta.value
        val tangentTheta = ArmKinematics.inverse(solutions[0]).theta.value

        tangentPoint = if(tangentTheta in startTheta..endTheta){ // Change this if a point on the circle causes problems
            solutions[0]
        }else{
            solutions[1]
        }

        return tangentPoint
    }

    private fun findIntersectionPoints(): Boolean{
        val m = path.getM()
        val b = path.getB()
        //println("m : $m, b : $b")
        val Ay = Math.pow(m, 2.0) + 1
        val By = 2 * b
        val Cy = Math.pow(b, 2.0) - Math.pow(m, 2.0) * Math.pow(r, 2.0)

        val Ax = Math.pow(m, 2.0) + 1
        val Bx = 2.0 * b * m
        val Cx = Math.pow(b, 2.0) - Math.pow(r, 2.0)

        if (Math.pow(By, 2.0) - 4 * Ay * Cy < 0 || Math.pow(Bx, 2.0) - 4 * Ax * Cx < 0){
            // No real solutions
            //println("No real solutions")
            return false
        }
        //val intY1 = -By + Math.sqrt(By * By - 4 * Ay * Cy) / (2 * Ay)
        //val intY2 = -By - Math.sqrt(By * By - 4 * Ay * Cy) / (2 * Ay)

        val intX1 = (-Bx + Math.sqrt(Math.pow(Bx, 2.0) - (4 * Ax * Cx))) / (2 * Ax)
        val intX2 = (-Bx - Math.sqrt((Bx * Bx) - (4 * Ax * Cx))) / (2 * Ax)

        //println("intX1 : $intX1, intX2 : $intX2")

        var min = x1.value
        var max = x2.value
        if (min > max){
            min = x2.value
            max = x1.value
        }

        if (intX1 !in min..max && intX2 !in min..max){
            //println("Out of scope")
            return false
        }
        return true
    }
    private fun withinCircle(point: Point2d): Boolean{
        val value = Math.sqrt(Math.pow(point.x.value, 2.0) + Math.pow(point.y.value, 2.0))
        if (Math.abs(value - r) < 0.1){
            return false
        }

        return value < r
    }
}
