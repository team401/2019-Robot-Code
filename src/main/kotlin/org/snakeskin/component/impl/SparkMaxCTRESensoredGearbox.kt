package org.snakeskin.component.impl

import com.ctre.phoenix.motorcontrol.IMotorController
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.Encoder
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.measure.MagEncoderTicks
import org.snakeskin.measure.MagEncoderTicksPerSecond
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureMagEncoderTicks
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureMagEncoderTicksPerHundredMilliseconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond

/**
 * @author Cameron Earle
 * @version 2/2/2019
 *
 */
class SparkMaxCTRESensoredGearbox(val encoder: Encoder, val master: CANSparkMax, vararg val slaves: CANSparkMax): ISmartGearbox<CANSparkMax> by SparkMaxGearbox(master, *slaves) {
    override fun getPosition(): AngularDistanceMeasureRadians {
        return (encoder.raw / 4096.0 * 2.0 * Math.PI).Radians
    }

    override fun getVelocity(): AngularVelocityMeasureRadiansPerSecond {
        return ((encoder.rate * 4.0) / 4096.0 * 2.0 * Math.PI).RadiansPerSecond
    }

    override fun setPosition(position: AngularDistanceMeasureRadians) {
        encoder.reset()
    }
}