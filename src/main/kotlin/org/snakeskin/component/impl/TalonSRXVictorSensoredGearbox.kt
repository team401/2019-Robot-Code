package org.snakeskin.component.impl

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import edu.wpi.first.wpilibj.Encoder
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.measure.Radians
import org.snakeskin.measure.RadiansPerSecond
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond

class TalonSRXVictorSensoredGearbox(val encoder: Encoder, val master: TalonSRX, vararg val slaves: VictorSPX) : ISmartGearbox<TalonSRX> by TalonSRXVictorGearbox(master, *slaves) {
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