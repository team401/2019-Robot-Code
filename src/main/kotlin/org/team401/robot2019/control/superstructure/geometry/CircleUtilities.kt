package org.team401.robot2019.control.superstructure.geometry

import org.snakeskin.measure.Inches
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.team401.robot2019.util.epsGe
import org.team401.robot2019.util.epsGt
import org.team401.robot2019.util.epsLt

/**
 * @author Cameron Earle
 * @version 2/27/2019
 *
 */
object CircleUtilities {
    /**
     * Returns true if the specified point lies within (exclusive) the circle from the origin to the specified radius
     */
    fun pointWithinCircle(r: LinearDistanceMeasureInches, point: Point2d): Boolean {
        val pointR = Math.hypot(point.x.value, point.y.value).Inches

        return pointR.value epsLt r.value
    }

    /**
     * Returns true if the specified point lies outside (inclusive) the circle from the origin to the specified radius
     */
    fun pointOutsideCircle(r: LinearDistanceMeasureInches, point: Point2d): Boolean {
        val pointR = Math.hypot(point.x.value, point.y.value).Inches

        return pointR.value epsGe r.value
    }

    /**
     * Identifies the two points on the circle which are intersected by the line tangent to the circle, which intersects the specified point.
     *
     * Ordering of the output array follows a few basic rules:
     * 1. The tangent point with the higher y value will always be the first element in the array
     * 2. If the y levels are equal (point is at x=0), the leftmost point will always be the first element in the array
     * 3. If there is only one tangent point, it will be both the first and second elements
     * 4. If the point is inside the radius of the circle, this is considered invalid and an empty array will be returned
     */
    fun identifyTangentPoints(r: LinearDistanceMeasureInches, point: Point2d): Array<Point2d> {
        val px = point.x.value
        val py = point.y.value
        val a = r.value

        val b = Math.hypot(px, py) //Distance from point to origin
        if (a epsGt b) return arrayOf() //Point is inside circle, invalid
        val th = Math.acos(a / b) //Angle between hypotenuse and side a
        val d = Math.atan2(py, px) //Direction from point to origin
        val d1 = d + th //Direction from t1 to origin
        val d2 = d - th //Direction from t1 to origin
        val t1x = a * Math.cos(d1) //Tangent 1 x
        val t1y = a * Math.sin(d1) //Tangent 1 y
        val t2x = a * Math.cos(d2) //Tangent 2 x
        val t2y = a * Math.sin(d2) //Tangent 2 y

        val t1 = Point2d(t1x.Inches, t1y.Inches)
        val t2 = Point2d(t2x.Inches, t2y.Inches)

        return if (px epsLt 0.0) {
            arrayOf(t2, t1) //If px is left of the origin, t2 is the positive tangent
        } else {
            arrayOf(t1, t2) //This handles both the 0 and above cases
        }
    }
}