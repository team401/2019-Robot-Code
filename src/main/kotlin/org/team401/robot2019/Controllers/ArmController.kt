package org.team401.robot2019.Controllers

import org.team401.robot2019.config.Geometry.ArmGeometery.maxX
import org.team401.robot2019.config.Geometry.ArmGeometery.maxY
import org.team401.robot2019.config.Geometry.ArmGeometery.minY
import org.team401.robot2019.config.Geometry.ArmGeometery.startExtension
import kotlin.math.abs

import kotlin.math.atan2
import kotlin.math.cos


object ArmController{
    //pwm = P * error + D * ((error - prev_error) / dt - goal_velocity) + Kv * goal_velocity + Ka * goal_acceleration
    fun generateRotationProfile(theta: Double, time: Int){

    }


    //return the arm extension and angle for reaching the desired position
    //All values in inches
    //TODO Write coordinate classes
    fun getArmState(x: Double, y: Double): Pair<Double, Double>{

        var x = x
        var y = y

        //check to see if the input is within range
        if (abs(x) > maxX.value){
            x = maxX.value
        }
        if (y < minY.value){
            y = minY.value
        }
        if (y > maxY.value){
            y = maxY.value
        }
        //Calculate Theta
        val theta = atan2(y, x) // from -Pi to Pi

        val r: Double = x/cos(theta) // Theta is in radians

        val extension: Double = r - startExtension.value

        return Pair(extension, theta)
    }

}