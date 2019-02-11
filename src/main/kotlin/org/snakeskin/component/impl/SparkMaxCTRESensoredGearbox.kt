package org.snakeskin.component.impl

import com.ctre.phoenix.motorcontrol.IMotorController
import com.revrobotics.CANSparkMax
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureMagEncoderTicks
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureMagEncoderTicksPerHundredMilliseconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond

/**
 * @author Cameron Earle
 * @version 2/2/2019
 *
 */
class SparkMaxCTRESensoredGearbox(val ctreController: IMotorController, val master: CANSparkMax, vararg val slaves: CANSparkMax): ISmartGearbox<CANSparkMax> by SparkMaxGearbox(master, *slaves) {
    override fun getPosition(): AngularDistanceMeasureRadians {
        return AngularDistanceMeasureMagEncoderTicks(ctreController.getSelectedSensorPosition(0).toDouble()).toRadians()
    }

    override fun getVelocity(): AngularVelocityMeasureRadiansPerSecond {
        return AngularVelocityMeasureMagEncoderTicksPerHundredMilliseconds(ctreController.getSelectedSensorVelocity(0).toDouble()).toRadiansPerSecond()
    }

    override fun setPosition(position: AngularDistanceMeasureRadians) {
        ctreController.setSelectedSensorPosition(position.toMagEncoderTicks().value.toInt(), 0, 0)
    }
}