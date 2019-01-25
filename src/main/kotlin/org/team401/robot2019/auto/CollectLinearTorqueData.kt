package org.team401.robot2019.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.units.Meters
import org.snakeskin.units.RadiansPerSecond
import org.snakeskin.units.Seconds
import org.snakeskin.units.measure.time.TimeMeasure
import org.team401.taxis.diffdrive.component.PathFollowingDiffDrive
import org.team401.taxis.util.Util

/**
 * @author Cameron Earle
 * @version 1/16/2019
 *
 */
class CollectLinearTorqueData(val drive: PathFollowingDiffDrive, val power: Double = 1.0, val runtime: TimeMeasure, val axis: Int = 0): AutoStep() {
    companion object {
        const val ACCEL_TO_MS2 = 0.0005985504150390625
        const val ACCEL_MINIMUM_MS2 = 1.0

        fun analyzeData(data: List<Pair<Double, Double>>, drive: PathFollowingDiffDrive): Double {
            val acceptedAccelerations = arrayListOf<Double>()
            var droppedPointCount = 0
            for (i in 0 until data.size) {
                val acceleration = data[i].second

                if (acceleration > ACCEL_MINIMUM_MS2) {
                    acceptedAccelerations.add(acceleration)
                } else {
                    droppedPointCount++
                }
            }
            println("Dropped $droppedPointCount data points")
            println("Accepted ${acceptedAccelerations.size} data points")
            val acceleration = acceptedAccelerations.average()
            println("Acceleration: $acceleration m/s/s")
            val torque = acceleration * drive.fullStateModel.drivetrainDynamicsModel.mass() * drive.wheelRadius.toUnit(Meters).value
            println("Torque: $torque N*m")
            return torque
        }

        fun createAuto(drive: PathFollowingDiffDrive, power: Double = 1.0, runtime: TimeMeasure, axis: Int = 0): RobotAuto {
            return object : RobotAuto(10L) {
                override fun assembleAuto(): SequentialSteps {
                    return SequentialSteps(CollectLinearTorqueData(drive, power, runtime, axis))
                }
            }
        }
    }

    val data = arrayListOf<Pair<Double, Double>>() //Time (sec), Velocity (rad/s)
    var startTime = 0.0
    val xyz = ShortArray(3)

    override fun entry(currentTime: Double) {
        startTime = currentTime
        drive.setNeutralMode(NeutralMode.Brake)
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        drive.arcade(ControlMode.PercentOutput, power, 0.0)
        drive.imu.getBiasedAccelerometer(xyz)
        data.add(currentTime to (xyz[axis] * ACCEL_TO_MS2))
        return currentTime - startTime >= runtime.toUnit(Seconds).value
    }

    override fun exit(currentTime: Double) {
        drive.stop()
        analyzeData(data, drive)
    }
}