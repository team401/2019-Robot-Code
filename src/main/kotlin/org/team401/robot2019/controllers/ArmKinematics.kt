package org.team401.robot2019.controllers

import org.snakeskin.units.Inches
import org.snakeskin.units.Radians
import org.snakeskin.units.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.units.measure.distance.linear.LinearDistanceMeasureInches
import java.lang.Math.pow
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

object ArmKinematics{

    private var x = 0.0.Inches
    private var y = 0.0.Inches
    private val pivotHeight = 18.75.Inches
    private var theta = 0.0.Radians
    private var radius = 0.0.Inches

    fun update(radius: LinearDistanceMeasureInches, theta: AngularDistanceMeasureRadians){
        this.theta = theta
        this.radius = radius
        this.x = (radius.value * cos(theta.value)).Inches
        this.y = ((radius.value * sin(theta.value)) + pivotHeight.value).Inches
    }

    fun getRectCoordinates(): Pair<LinearDistanceMeasureInches, LinearDistanceMeasureInches>{
        return Pair(x, y)
    }

    /**
     * Returns current polar coordinates
     */
    fun getPolarCoordinates():Pair<LinearDistanceMeasureInches, AngularDistanceMeasureRadians> {
        return Pair(radius, theta)
    }


    fun getPolarCoordinates(x: LinearDistanceMeasureInches, y: LinearDistanceMeasureInches)
            : Pair<LinearDistanceMeasureInches, AngularDistanceMeasureRadians>{
        val theta = atan2(y.value, x.value).Radians
        val radius = sqrt(pow(x.value, 2.0) + pow(y.value, 2.0)).Inches
        return Pair(radius, theta)
    }

    fun toStringPolar():String{
        return "(r : ${radius.value}, theta : ${theta.value})"
    }

    override fun toString(): String {
        return "(x : ${x.value}, y : ${y.value})"
    }
}