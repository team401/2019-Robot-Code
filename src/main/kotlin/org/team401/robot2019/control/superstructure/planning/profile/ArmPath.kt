package org.team401.robot2019.control.superstructure.planning.profile

import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.geometry.PointPolar
import org.team401.robot2019.subsystems.arm.control.ArmKinematics

class ArmPath(private val path: LinearProfileSegment, minimumRadius: LinearDistanceMeasureInches){
    // Restricts the bounds of the start and end
    private val start = path.start
    private val end = path.end

    private val y1 = path.start.y
    private val y2 = path.end.y
    private val x1 = path.start.x
    private val x2 = path.end.x

    private val r = minimumRadius.value
    //private val a = y1 - y2
    //private val b = x2 - x1
    //private val c = (x1 - x2) * y1 + (y2 - y1) * x1


    fun solve(): Array<ProfileSegment>{
        //println("Solving")
        val intersectsCircle = findIntersectionPoints()
        //println("intersects Circle : $intersectsCircle")
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

        if (intersectsCircle) {
            //println("Intersects Circle")
            val tangentPointOne = findTangentPoint(x1.value, y1.value)
            val tangentPointTwo = findTangentPoint(x2.value, y2.value)

            val firstSegment = LinearProfileSegment(start, tangentPointOne)
            val secondSegment = ArcProfileSegment(tangentPointOne, tangentPointTwo, r)
            val thirdSegment = LinearProfileSegment(tangentPointTwo, end)
            //println("1st tangent : $tangentPointOne, 2nd tangent: $tangentPointTwo")
            //println("end : $end")

            segments = arrayOf(firstSegment, secondSegment, thirdSegment)

        }else{
            segments = arrayOf(LinearProfileSegment(start, end))
        }
        //segments.forEach { println("${it.start}, ${it.end}") }

        return segments
    }

    private fun findTangentPoint(x1: Double, y1: Double): Point2d {
        // x coordinate of tangent position
        val B = -2 * x1 * Math.pow(r, 2.0)
        val A = (Math.pow(y1, 2.0) + Math.pow(x1, 2.0))
        val C =  -(Math.pow(y1, 2.0) * Math.pow(r, 2.0)) + (Math.pow(r, 4.0))
        /*
        val d1 = (2 * x1 * Math.pow(r, 2.0) + Math.sqrt(Math.pow(2 * x1 * Math.pow(r, 2.0), 2.0)
                - 4 * ((Math.pow(y1, 2.0) + Math.pow(x1, 2.0)) * (Math.pow(r, 4.0) - (Math.pow(y1, 2.0) * Math.pow(r, 2.0))))))/ (2 * (Math.pow(y1, 2.0) + Math.pow(x1, 2.0)))


        val d2 = (2 * x1 * Math.pow(r, 2.0) - Math.sqrt(Math.pow(2 * x1 * Math.pow(r, 2.0), 2.0)
                - 4 * ((Math.pow(y1, 2.0) + Math.pow(x1, 2.0)) * (Math.pow(r, 4.0) - (Math.pow(y1, 2.0) * Math.pow(r, 2.0))))))/ (2 * (Math.pow(y1, 2.0) + Math.pow(x1, 2.0)))
        */

        val discriminant = Math.pow(B, 2.0) - 4 * A * C
        var d1: Double
        var d2: Double

        if( discriminant < 0.0 && discriminant >= -0.000001) {
            d1 = -B / (2 * A)
            d2 = d1
        }else{
            d1 = (-B + Math.sqrt(Math.pow(B, 2.0) - 4 * A * C))/ (2 * A)
            d2 = (-B - Math.sqrt(Math.pow(B, 2.0) - 4 * A * C))/ (2 * A)
        }

        // y coordinate of tangent position
        var e1: Double
        var e2: Double
        if(y1 >= 0.001 || y1 <= -0.001){
        //if (y1 != 0.0) { // There is a plus or minus here, adjust if it becomes a problem
            //println("y1 != 0.0")
            e1 = (Math.pow(r, 2.0) - (x1 * d1)) / y1
            e2 = (Math.pow(r, 2.0) - (x1 * d2)) / y1
        }else{
            //println("y1 == 0.0")
            e1 = Math.sqrt(x1 * d1 - Math.pow(d1, 2.0))
            e2 = Math.sqrt(x1 * d2 - Math.pow(d2, 2.0))
        }

        //println("x1 : $x1, x2 : $x2")
        //if(d2 < 0 && this.x1 < this.x2){
        //    e1 *= -1
        //}

        //println("d1 : $d1, d2: $d2")
        //println("e1 : $e1, e2: $e2")

        var d: Double
        var e: Double
        /*
        if(this.x1 < this.x2 || e1 < e2){
            d = d2
            e = e2
        }else{
            d = d1
            e = e1
        }
        */
        if(e1 <= e2){ // Going "backwards"
            e = e2
            if(withinTolerance(x1, 0.0, 0.0001)) {
                if (this.x1 < this.x2) {
                    d = if (d1 < d2) {
                        d2
                    } else {
                        d1
                    }
                } else {
                    d = if (d1 < d2) {
                        d1
                    } else {
                        d2
                    }
                }
            }else{
                d = d2
            }
        }else{
            e = e1
            d = d1
        }
        //println("d : $d, e : $e")

        //print("Distance from origin to Tan position : ${Math.sqrt(Math.pow(d, 2.0) + Math.pow(e, 2.0))}")

        return Point2d(d.Inches, e.Inches)
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

        if (!(intX1 >= min && intX1 <= max) && !(intX2 >= min && intX2 <= max)){
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
    private fun withinTolerance(value: Double, target: Double, tolerance: Double): Boolean{
        return Math.abs(value - target) < Math.abs(tolerance)

    }
}
