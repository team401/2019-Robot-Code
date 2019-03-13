package org.team401.robot2019.control.climbing

import org.snakeskin.logic.LockingDelegate
import org.snakeskin.measure.Inches
import org.snakeskin.measure.InchesPerSecond
import org.snakeskin.measure.Seconds
import org.snakeskin.measure.acceleration.linear.LinearAccelerationMeasureInchesPerSecondPerSecond
import org.snakeskin.measure.distance.linear.LinearDistanceMeasureInches
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.snakeskin.measure.velocity.linear.LinearVelocityMeasureInchesPerSecond
import java.lang.Math.abs

/**
 * @author Eli Jelesko
 * @version 1/19/2019
 *
 * Stolen by: Cameron Earle
 * Just the profiler from the arm changed to linear units for climbing
 */
class ClimbingProfileGenerator(maxVelocity: LinearVelocityMeasureInchesPerSecond,
                                  maxAcceleration: LinearAccelerationMeasureInchesPerSecondPerSecond,
                                  start: LinearDistanceMeasureInches,
                                  end: LinearDistanceMeasureInches){
    // Discrete Form : Pt = Pt + Vt + 1/2A
    // Continuous Form : Pt = P0 + VoT + 1/2AT^2
    private val startPos = start.value
    private val endPos = end.value

    private val backwards = startPos > endPos

    private var position = startPos
    private var velocity = 0.0 // Units/sec
    private val maxVel = if (backwards){-maxVelocity.value}else{maxVelocity.value} // Units/sec/sec
    private val maxAccel = if (backwards){-maxAcceleration.value}else{maxAcceleration.value}

    private val points = ArrayList<ClimbingProfilePoint>()

    private var done by LockingDelegate(false)
    private var firstPhaseDone by LockingDelegate(false)
    private var secondPhaseDone by LockingDelegate(false)
    private var thirdPhaseDone by LockingDelegate(false)

    private var tempPoints = points.toList()
    private var i = 0

    var time = 0.0

    fun reset(){
        time = 0.0
        position = 0.0
        velocity = 0.0
        points.clear()
    }

    fun isDone(): Boolean{
        return done
    }

    /**
     * Called in a loop, generates and executes a profile
     * @param dt The time delta between now and the last time the function was called, in seconds
     */
    fun update(dt: Double): ClimbingProfilePoint {
        //println("update started")
        //println("start theta : $startPos, end theta : $endPos")
        when {
            !firstPhaseDone -> {
                if((position < startPos + ((endPos - startPos) / 2.0) && !backwards) ||
                        (position > endPos + ((startPos - endPos)/2.0)) && backwards){ // Generate the first half of the profile
                    //println("Pos: $position, Vel: $velocity, Time: $time")
                    time += dt

                    if (abs(velocity) < abs(maxVel)){
                        velocity += maxAccel * dt
                    }else{
                        velocity = maxVel
                    }

                    position += velocity * dt

                    points.add(
                        ClimbingProfilePoint(
                            position.Inches,
                            velocity.InchesPerSecond,
                            time.Seconds
                        )
                    )
                }else{
                    firstPhaseDone = true
                }
            }
            !secondPhaseDone -> {
                tempPoints = points.toList()
                i = tempPoints.size - 2
                secondPhaseDone = true

                if (points.size < 2 * tempPoints.size - 1){
                    //println("Pos: $position, Vel: $velocity, Time: $time")

                    time += dt

                    velocity = tempPoints[i].velocity.value
                    i--

                    position += velocity * dt


                    points.add(
                        ClimbingProfilePoint(
                            position.Inches,
                            velocity.InchesPerSecond,
                            time.Seconds
                        )
                    )
                }
            }
            !thirdPhaseDone -> {
                if (points.size < 2 * tempPoints.size - 1){
                    //println("Pos: $position, Vel: $velocity, Time: $time")

                    time += dt

                    velocity = tempPoints[i].velocity.value
                    i--

                    position += velocity * dt
                    if((position > endPos && !backwards) || (position < endPos) && backwards){ // This might cause problems
                        position = endPos
                    }

                    points.add(
                        ClimbingProfilePoint(
                            position.Inches,
                            velocity.InchesPerSecond,
                            time.Seconds
                        )
                    )
                }else{
                    thirdPhaseDone = true
                }
            }
            else -> {
                done = true
            }
        }
        //println("end : $endPos, lastPoint: ${points.last().position}")
        return points.last()
    }

    fun getPosition(): Array<LinearDistanceMeasureInches>{
        return Array(points.size){points[i].position}
    }

    fun getTime(): Array<TimeMeasureSeconds>{
        return Array(points.size){points[i].time}
    }
}