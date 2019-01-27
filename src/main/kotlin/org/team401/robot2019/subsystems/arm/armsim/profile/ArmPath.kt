package org.team401.armsim.profile

import org.team401.armsim.Point2d
import java.lang.Math.pow
import kotlin.math.*

class ArmPath(path: LinearProfileSegment){

    // Restricts the bounds of the start and end
    private val path = path
    private val start = path.start
    private val end = path.end

    private val y1 = path.start.y
    private val y2 = path.end.y
    private val x1 = path.start.x
    private val x2 = path.end.x

    private val r = 4.0
    //private val a = y1 - y2
    //private val b = x2 - x1
    //private val c = (x1 - x2) * y1 + (y2 - y1) * x1


    fun solve(): Array<ProfileSegment>{
        println("Distance ")
        val intersectsCircle = findIntersectionPoints()
        println(intersectsCircle)
        Thread.sleep(100)
        var segments: Array<ProfileSegment>

        if (intersectsCircle) {
            println("Intersects Circle")
            val tangentPointOne = findTangentPoint(x1, y1)
            val tangentPointTwo = findTangentPoint(x2, y2)

            val firstSegment = LinearProfileSegment(start, tangentPointOne)
            val secondSegment = ArcProfileSegment(tangentPointOne, tangentPointTwo, r)
            val thirdSegment = LinearProfileSegment(tangentPointTwo, end)

            segments = arrayOf(firstSegment, secondSegment, thirdSegment)

        }else{
            segments = arrayOf(LinearProfileSegment(start, end))
        }
        //segments.forEach { println("${it.start}, ${it.end}") }

        return segments
    }

    private fun findTangentPoint(x1: Double, y1: Double): Point2d {
        // x coordinate of tangent point
        val d1 = (2 * x1 * pow(r, 2.0) + sqrt(pow(2 * x1 * pow(r, 2.0), 2.0)
                - 4 * ((pow(y1, 2.0) + pow(x1, 2.0)) * (pow(r, 4.0) - (pow(y1, 2.0) * pow(r, 2.0))))))/ (2 * (pow(y1, 2.0) + pow(x1, 2.0)))


        val d2 = (2 * x1 * pow(r, 2.0) - sqrt(pow(2 * x1 * pow(r, 2.0), 2.0)
                - 4 * ((pow(y1, 2.0) + pow(x1, 2.0)) * (pow(r, 4.0) - (pow(y1, 2.0) * pow(r, 2.0))))))/ (2 * (pow(y1, 2.0) + pow(x1, 2.0)))

        // y coordinate of tangent point
        var e1: Double
        var e2: Double
        if (y1 != 0.0) { // There is a plus or minus here, adjust if it becomes a problem
            e1 = (pow(r, 2.0) - x1 * d1) / y1
            e2 = (pow(r, 2.0) - x1 * d2) / y1
        }else{
            e1 = sqrt(x1 * d1 - pow(d1, 2.0))
            e2 = sqrt(x1 * d2 - pow(d2, 2.0))
        }

        var d: Double
        var e: Double
        if(e2 > e1){
            d = d2
            e = e2
        }else{
            d = d1
            e = e1
        }
        //println("d : $d, e: $e")
        print("Distance from origin to Tan point : ${sqrt(pow(d, 2.0) + pow(e, 2.0))}")
        Thread.sleep(100)

        return Point2d(d, e)
    }

    private fun findIntersectionPoints(): Boolean{
        val m = path.getM()
        val b = path.getB()
        val Ay = pow(m, 2.0) + 1
        val By = 2 * b
        val Cy = pow(b, 2.0) - pow(m, 2.0) * pow(r, 2.0)

        val Ax = pow(m, 2.0) + 1
        val Bx = 2 * b * m
        val Cx = pow(b, 2.0) - pow(r, 2.0)

        if (pow(By, 2.0) - 4 * Ay * Cy < 0 || pow(Bx, 2.0) - 4 * Ax * Cx < 0){
            // No real solutions
            println("No real solutions")
            return false
        }
        val intY1 = -By + sqrt(By * By - 4 * Ay * Cy) / (2 * Ay)
        val intY2 = -By - sqrt(By * By - 4 * Ay * Cy) / (2 * Ay)

        val intX1 = -Bx + sqrt(Bx * Bx - 4 * Ax * Cx) / (2 * Ax)
        val intX2 = -Bx - sqrt(Bx * Bx - 4 * Ax * Cx) / (2 * Ax)

        var min = x1
        var max = x2
        if (min > max){
            min = x2
            max = x1
        }

        if (!(intX1 >= min && intX1 <= max) && !(intX2 >= min && intX2 <= max)){
            println("Out of scope")
            return false
        }
        return true
    }
}