package org.team401.robot2019.auto

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.AutoStep
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.hardware.Hardware
import org.snakeskin.measure.time.TimeMeasureSeconds
import org.team401.robot2019.config.Physics
import org.team401.taxis.diffdrive.component.IPathFollowingDiffDrive
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.util.Util

/**
 * @author Cameron Earle
 * @version 1/18/2019
 *
 */
class CollectAngularTorqueData(val drive: IPathFollowingDiffDrive<ISmartGearbox<*>>, val power: Double, val runtime: TimeMeasureSeconds, val linearTorque: Double, val axis: Int = 0): AutoStep() {
    companion object {
        const val ACCEL_MINIMUM_RADS2 = 1.0

        fun analyzeData(data: List<Pair<Double, Double>>, drive: IPathFollowingDiffDrive<ISmartGearbox<*>>, linearTorque: Double): Double {
            val acceptedAccelerations = arrayListOf<Double>()
            var droppedPointCount = 0

            for (i in 1 until data.size) {
                val currentTime = data[i].first
                val lastTime = data[i-1].first
                val currentAngVel = data[i].second // rad/s
                val lastAngVel = data[i-1].second // rad/s

                val acceleration = Math.abs((currentAngVel - lastAngVel) / (currentTime - lastTime)) // rad/s/s
                if (acceleration > ACCEL_MINIMUM_RADS2) {
                    acceptedAccelerations.add(acceleration)
                } else {
                    droppedPointCount++
                }
            }

            println("Dropped $droppedPointCount data points")
            println("Accepted ${acceptedAccelerations.size} data points")
            val avgAngAccel = acceptedAccelerations.average() // rad/s/s
            println("Average Angular Acceleration (rad/s/s): $avgAngAccel")
            val moi = (linearTorque / drive.wheelRadius.toMeters().value * ((drive.wheelbase.toMeters().value * Physics.DrivetrainDynamics.trackScrubFactor) / 2.0)) / avgAngAccel
            println("Robot MOI: $moi")
            println("$avgAngAccel\t$moi")
            return moi
        }

        fun createAuto(drive: IPathFollowingDiffDrive<ISmartGearbox<*>>, power: Double = 1.0, runtime: TimeMeasureSeconds, linearTorque: Double, axis: Int = 0): RobotAuto {
            return object : RobotAuto(10L) {
                override fun assembleAuto(): SequentialSteps {
                    return SequentialSteps(CollectAngularTorqueData(drive, power, runtime, linearTorque, axis))
                }
            }
        }
    }


    val data = arrayListOf<Pair<Double, Double>>() //Time (sec), Angle (rad)
    var startTime = 0.0
    val xyzDps = DoubleArray(3)

    override fun entry(currentTime: Double) {
        drive.both {
            setRampRate(0.0)
            setNeutralMode(ISmartGearbox.CommonNeutralMode.BRAKE)
        }
        startTime = currentTime
    }

    override fun action(currentTime: Double, lastTime: Double): Boolean {
        drive.arcade(0.0, power)
        drive.imu.getRawGyro(xyzDps)
        data.add(currentTime to xyzDps[axis] * (Math.PI / 180.0))
        return currentTime - startTime >= runtime.toSeconds().value
    }

    override fun exit(currentTime: Double) {
        drive.stop()
        analyzeData(data, drive, linearTorque)
    }
}