package org.team401.robot2019.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.units.MagEncoderTicksPer100Ms
import org.snakeskin.units.Seconds
import org.snakeskin.units.measure.time.TimeMeasure
import org.team401.taxis.diffdrive.component.PathFollowingDiffDrive

/**
 * @author Cameron Earle
 * @version 1/19/2019
 *
 */
class CollectBasicFeedForward(val drive: PathFollowingDiffDrive, val accelTime: TimeMeasure, val power: Double = 1.0, val samples: Int = 100): AutoStep() {
    var counter = 0
    var startTime = 0.0
    val dataLeft = arrayListOf<Double>()
    val dataRight = arrayListOf<Double>()

    companion object {
        fun analyzeData(dataLeft: List<Double>, dataRight: List<Double>, power: Double): Pair<Double, Double> {
            val avgSampleVelLeft = dataLeft.average()
            val avgSampleVelRight = dataRight.average()

            println("Left Avg Vel (nu): $avgSampleVelLeft")
            println("Right Avg Vel (nu): $avgSampleVelRight")
            println("90% Avg vel target (nu): ${((avgSampleVelLeft + avgSampleVelRight) / 2.0) * 0.90}")

            val leftFF = (power * 1023.0) / avgSampleVelLeft
            val rightFF = (power * 1023.0) / avgSampleVelRight

            println("Left FF: $leftFF")
            println("Right FF: $rightFF")

            return leftFF to rightFF
        }

        fun createAuto(drive: PathFollowingDiffDrive, accelTime: TimeMeasure, power: Double = 1.0, samples: Int = 100): RobotAuto {
            return object : RobotAuto(10L) {
                override fun assembleAuto(): SequentialSteps {
                    return SequentialSteps(CollectBasicFeedForward(drive, accelTime, power, samples))
                }
            }
        }
    }

    override fun entry(currentTime: Double) {
        counter = 0
        startTime = currentTime
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        drive.arcade(ControlMode.PercentOutput, power, 0.0)
        if (currentTime - startTime >= accelTime.toUnit(Seconds).value) { //Done accelerating, start taking samples
            dataLeft.add(drive.left.getVelocity().toUnit(MagEncoderTicksPer100Ms).value)
            dataRight.add(drive.right.getVelocity().toUnit(MagEncoderTicksPer100Ms).value)
            counter++
        }

        return counter >= samples
    }

    override fun exit(currentTime: Double) {
        drive.stop()
        analyzeData(dataLeft, dataRight, power)
    }
}