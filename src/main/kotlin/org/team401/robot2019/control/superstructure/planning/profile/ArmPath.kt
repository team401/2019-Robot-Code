package org.team401.robot2019.control.superstructure.planning.profile

import org.snakeskin.measure.Inches
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.config.Geometry

class ArmPath(path: LinearProfileSegment){

    // Restricts the bounds of the start and end
    private val path = path
    private val start = path.start
    private val end = path.end

    private val y1 = path.start.y
    private val y2 = path.end.y
    private val x1 = path.start.x
    private val x2 = path.end.x

    private val r = Geometry.ArmGeometry.minSafeWristToolChangeRadius.value
    //private val a = y1 - y2
    //private val b = x2 - x1
    //private val c = (x1 - x2) * y1 + (y2 - y1) * x1


    fun solve(): Array<ProfileSegment>{
        //println("Distance ")
        val intersectsCircle = findIntersectionPoints()
        println(intersectsCircle)
        Thread.sleep(100)
        var segments: Array<ProfileSegment>

        if (intersectsCircle) {
            //println("Intersects Circle")
            val tangentPointOne = findTangentPoint(x1.value, y1.value)
            val tangentPointTwo = findTangentPoint(x2.value, y2.value)

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
        // x coordinate of tangent position
        val d1 = (2 * x1 * Math.pow(r, 2.0) + Math.sqrt(Math.pow(2 * x1 * Math.pow(r, 2.0), 2.0)
                - 4 * ((Math.pow(y1, 2.0) + Math.pow(x1, 2.0)) * (Math.pow(r, 4.0) - (Math.pow(y1, 2.0) * Math.pow(r, 2.0))))))/ (2 * (Math.pow(y1, 2.0) + Math.pow(x1, 2.0)))


        val d2 = (2 * x1 * Math.pow(r, 2.0) - Math.sqrt(Math.pow(2 * x1 * Math.pow(r, 2.0), 2.0)
                - 4 * ((Math.pow(y1, 2.0) + Math.pow(x1, 2.0)) * (Math.pow(r, 4.0) - (Math.pow(y1, 2.0) * Math.pow(r, 2.0))))))/ (2 * (Math.pow(y1, 2.0) + Math.pow(x1, 2.0)))

        // y coordinate of tangent position
        var e1: Double
        var e2: Double
        if (y1 != 0.0) { // There is a plus or minus here, adjust if it becomes a problem
            e1 = (Math.pow(r, 2.0) - x1 * d1) / y1
            e2 = (Math.pow(r, 2.0) - x1 * d2) / y1
        }else{
            e1 = Math.sqrt(x1 * d1 - Math.pow(d1, 2.0))
            e2 = Math.sqrt(x1 * d2 - Math.pow(d2, 2.0))
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
        //print("Distance from origin to Tan position : ${Math.sqrt(Math.pow(d, 2.0) + Math.pow(e, 2.0))}")
        Thread.sleep(100)

        return Point2d(d.Inches, e.Inches)
    }

    private fun findIntersectionPoints(): Boolean{
        val m = path.getM()
        val b = path.getB()
        val Ay = Math.pow(m, 2.0) + 1
        val By = 2 * b
        val Cy = Math.pow(b, 2.0) - Math.pow(m, 2.0) * Math.pow(r, 2.0)

        val Ax = Math.pow(m, 2.0) + 1
        val Bx = 2 * b * m
        val Cx = Math.pow(b, 2.0) - Math.pow(r, 2.0)

        if (Math.pow(By, 2.0) - 4 * Ay * Cy < 0 || Math.pow(Bx, 2.0) - 4 * Ax * Cx < 0){
            // No real solutions
            //println("No real solutions")
            return false
        }
        //val intY1 = -By + Math.sqrt(By * By - 4 * Ay * Cy) / (2 * Ay)
        //val intY2 = -By - Math.sqrt(By * By - 4 * Ay * Cy) / (2 * Ay)

        val intX1 = -Bx + Math.sqrt(Bx * Bx - 4 * Ax * Cx) / (2 * Ax)
        val intX2 = -Bx - Math.sqrt(Bx * Bx - 4 * Ax * Cx) / (2 * Ax)

        var min = x1.value
        var max = x2.value
        if (min > max){
            min = x2.value
            max = x1.value
        }

        if (!(intX1 >= min && intX1 <= max) && !(intX2 >= min && intX2 <= max)){
            println("Out of scope")
            return false
        }
        return true
    }
}