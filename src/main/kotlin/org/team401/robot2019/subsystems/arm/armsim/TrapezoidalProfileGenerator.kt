package org.team401.armsim

import org.snakeskin.logic.LockingDelegate
import kotlin.math.abs

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
class TrapezoidalProfileGenerator(maxVelocity: Double, maxAcceleration: Double, start: Double, end:Double){

    // Discrete Form : Pt = Pt + Vt + 1/2A
    // Continuous Form : Pt = P0 + VoT + 1/2AT^2
    private val startPos = start
    private val endPos = end

    private val backwards = startPos > endPos

    private val rate = 0.01
    private var position = startPos
    private var velocity = 0.0 // Units/sec
    private val maxVel = if (backwards){-maxVelocity}else{maxVelocity} // Units/sec/sec
    private val maxAccel = if (backwards){-maxAcceleration}else{maxAcceleration}

    private val points = ArrayList<Array<Double>>()

    private var done by LockingDelegate(false)
    private var firstPhaseDone by LockingDelegate(false)
    private var secondPhaseDone by LockingDelegate(false)
    private var thirdPhaseDone by LockingDelegate(false)

    private var tempPoints = points.toList()
    private var i = 0

    var time = 0.0

    /*
    fun generate(){
        //println("Start : $startPos End : $endPos")
        points.add(arrayOf(position, velocity, time))
        while((position < startPos + ((endPos - startPos) / 2.0) && !backwards) ||
            (position > endPos + ((startPos - endPos)/2.0)) && backwards){ // Generate the first half of the profile
            //println("Pos: $position, Vel: $velocity, Time: $time")
            time += rate

            if (abs(velocity) < abs(maxVel)){
                velocity += maxAccel * rate
            }else{
                velocity = maxVel
            }

            position += velocity * rate

            points.add(arrayOf(position, velocity, time))
        }

        val temp = points.toList()
        var i = temp.size - 2
        //points.removeAt(points.size -1)

        //println("${startPos - (startPos - endPos)/2.0}")
        //println("Second stage entered")
        while (points.size < 2 * temp.size - 1){
            println("Pos: $position, Vel: $velocity, Time: $time")

            time += rate

            velocity = temp[i][1]
            i--

            position += velocity * rate

            points.add(arrayOf(position, velocity, time))
        }
        //println("End theta : $endPos")
        //println("Final Pos: $position, Vel: $velocity, Time: $time")
        //println("Points length: ${points.size}")
        //println("Temp length : ${temp.size}")
    }
    */
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
     * Called in a loop, generates a profile
     */
    fun updatePoint(): Array<Double>{
        when {
            !firstPhaseDone -> {
                if((position < startPos + ((endPos - startPos) / 2.0) && !backwards) ||
                        (position > endPos + ((startPos - endPos)/2.0)) && backwards){ // Generate the first half of the profile
                    //println("Pos: $position, Vel: $velocity, Time: $time")
                    time += rate

                    if (abs(velocity) < abs(maxVel)){
                        velocity += maxAccel * rate
                    }else{
                        velocity = maxVel
                    }

                    position += velocity * rate

                    points.add(arrayOf(position, velocity, time))
                }else{
                    firstPhaseDone = true
                }
            }
            !secondPhaseDone -> {
                tempPoints = points.toList()
                i = tempPoints.size - 2
                secondPhaseDone = true

                if (points.size < 2 * tempPoints.size - 1){
                    println("Pos: $position, Vel: $velocity, Time: $time")

                    time += rate

                    velocity = tempPoints[i][1]
                    i--

                    position += velocity * rate

                    points.add(arrayOf(position, velocity, time))
                }
            }
            !thirdPhaseDone -> {
                if (points.size < 2 * tempPoints.size - 1){
                    println("Pos: $position, Vel: $velocity, Time: $time")

                    time += rate

                    velocity = tempPoints[i][1]
                    i--

                    position += velocity * rate

                    points.add(arrayOf(position, velocity, time))
                }else{
                    thirdPhaseDone = true
                }
            }
            else -> {
                done = true
            }
        }
        return points.last()
    }

    fun getPosition(): DoubleArray{
        val output = ArrayList<Double>()
        points.forEach {output.add(it[0])}
        return output.toDoubleArray()
    }

    fun getTime(): DoubleArray{
        val output = ArrayList<Double>()
        points.forEach {output.add(it[2])}
        return output.toDoubleArray()
    }
    /*
    fun graph(graphVelocity: Boolean){
        val positionSeries = DoubleArray(points.size){points[it][0]}
        val velocitySeries = DoubleArray(points.size){points[it][1]}
        val timeSeries = DoubleArray(points.size){points[it][2]}

        val positionGraph = QuickChart.getChart("Position vs Time", "Time", "Position", "x(t)", timeSeries, positionSeries)
        val velocityGraph = QuickChart.getChart("Velocity vs Time", "Time", "Velocity", "v(t)", timeSeries, velocitySeries)

        SwingWrapper(positionGraph).displayChart()
        if (graphVelocity){
            SwingWrapper(velocityGraph).displayChart()
        }
    }
    */
}