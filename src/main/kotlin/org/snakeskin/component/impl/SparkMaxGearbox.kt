package org.snakeskin.component.impl

import com.revrobotics.CANError
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import org.snakeskin.component.ISensoredGearbox
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRevolutions
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRevolutionsPerMinute
import org.snakeskin.template.PIDFTemplate

/**
 * @author Cameron Earle
 * @version 2/2/2019
 *
 */
class SparkMaxGearbox(val master: CANSparkMax, vararg val slaves: CANSparkMax): ISmartGearbox<CANSparkMax> {
    val encoder = master.encoder
    val pidController = master.pidController

    init {
        link()
    }

    override fun link() {
        slaves.forEach {
            it.follow(master)
        }
    }

    override fun getMasterOutputCurrent(): Double {
        return master.outputCurrent
    }

    override fun getMasterVbus(): Double {
        return master.busVoltage
    }

    override fun getOutputPercent(): Double {
        return master.get()
    }

    override fun getOutputVoltage(): Double {
        return master.appliedOutput
    }

    override fun set(controlMode: ISmartGearbox.CommonControlMode, setpoint: Double) {
        when (controlMode) {
            ISmartGearbox.CommonControlMode.PERCENTAGE -> master.set(setpoint)
            ISmartGearbox.CommonControlMode.VELOCITY -> TODO("Add")
            ISmartGearbox.CommonControlMode.POSITION -> TODO("Add")
        }
    }

    override fun set(controlMode: ISmartGearbox.CommonControlMode, setpoint: Double, arbFF: Double) {
        TODO("Add")
    }

    override fun setDeadband(deadbandPercent: Double): Boolean {
        return master.setParameter(CANSparkMaxLowLevel.ConfigParameter.kInputDeadband, deadbandPercent) == CANSparkMaxLowLevel.ParameterStatus.kOK
    }

    override fun setNeutralMode(mode: ISmartGearbox.CommonNeutralMode): Boolean {
        when (mode) {
            ISmartGearbox.CommonNeutralMode.BRAKE -> {
                master.idleMode = CANSparkMax.IdleMode.kBrake
                slaves.forEach {
                    it.idleMode = CANSparkMax.IdleMode.kBrake
                }
            }

            ISmartGearbox.CommonNeutralMode.COAST -> {
                master.idleMode = CANSparkMax.IdleMode.kCoast
                slaves.forEach {
                    it.idleMode = CANSparkMax.IdleMode.kCoast
                }
            }
        }

        return true
    }

    override fun setPIDF(kP: Double, kI: Double, kD: Double, kF: Double, slot: Int): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setPIDF(template: PIDFTemplate, slot: Int): Boolean {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun setRampRate(secondsFromNeutralToFull: Double): Boolean {
        return master.setRampRate(secondsFromNeutralToFull) == CANError.kOK
    }

    override var inverted: Boolean
        get() = master.inverted && slaves.all { it.inverted }
        set(value) {
            master.inverted = value
            slaves.forEach {
                it.inverted = value
            }
        }

    override fun get(): Double {
        return master.get()
    }

    override fun set(setpoint: Double) {
        master.set(setpoint)
    }

    override fun stop() {
        set(0.0)
    }

    override fun getPosition(): AngularDistanceMeasureRadians {
        return AngularDistanceMeasureRevolutions(master.encoder.position).toRadians()
    }

    override fun getVelocity(): AngularVelocityMeasureRadiansPerSecond {
        return AngularVelocityMeasureRevolutionsPerMinute(master.encoder.velocity).toRadiansPerSecond()
    }

    override fun setPosition(position: AngularDistanceMeasureRadians) {
        TODO("hurry up rev")
    }
}