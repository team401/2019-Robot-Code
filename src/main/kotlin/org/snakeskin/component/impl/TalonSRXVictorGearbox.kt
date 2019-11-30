package org.snakeskin.component.impl

import com.ctre.phoenix.ErrorCode
import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import org.snakeskin.component.ISmartGearbox
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureMagEncoderTicks
import org.snakeskin.measure.distance.angular.AngularDistanceMeasureRadians
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureMagEncoderTicksPerHundredMilliseconds
import org.snakeskin.measure.velocity.angular.AngularVelocityMeasureRadiansPerSecond
import org.snakeskin.template.PIDFTemplate

class TalonSRXVictorGearbox(val master: TalonSRX, vararg val slaves: VictorSPX): ISmartGearbox<TalonSRX> {
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
        return master.motorOutputPercent
    }

    override fun getOutputVoltage(): Double {
        return master.motorOutputVoltage
    }

    override fun set(controlMode: ISmartGearbox.CommonControlMode, setpoint: Double) {
        when (controlMode) {
            ISmartGearbox.CommonControlMode.PERCENTAGE -> master.set(ControlMode.PercentOutput, setpoint)
            ISmartGearbox.CommonControlMode.VELOCITY -> TODO("Add")
            ISmartGearbox.CommonControlMode.POSITION -> TODO("Add")
        }
    }

    override fun set(controlMode: ISmartGearbox.CommonControlMode, setpoint: Double, arbFF: Double) {
        TODO("Add")
    }

    override fun setDeadband(deadbandPercent: Double): Boolean {
        return master.configNeutralDeadband(deadbandPercent) == ErrorCode.OK
    }

    override fun setNeutralMode(mode: ISmartGearbox.CommonNeutralMode): Boolean {
        when (mode) {
            ISmartGearbox.CommonNeutralMode.BRAKE -> {
                master.setNeutralMode(NeutralMode.Brake)
                slaves.forEach {
                    it.setNeutralMode(NeutralMode.Brake)
                }
            }

            ISmartGearbox.CommonNeutralMode.COAST -> {
                master.setNeutralMode(NeutralMode.Coast)
                slaves.forEach {
                    it.setNeutralMode(NeutralMode.Coast)
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
        return master.configOpenloopRamp(secondsFromNeutralToFull) == ErrorCode.OK &&
                master.configClosedloopRamp(secondsFromNeutralToFull) == ErrorCode.OK
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
        return master.motorOutputPercent
    }

    override fun set(setpoint: Double) {
        master.set(ControlMode.PercentOutput, setpoint  )
    }

    override fun stop() {
        set(0.0)
    }

    override fun getPosition(): AngularDistanceMeasureRadians {
        return AngularDistanceMeasureMagEncoderTicks(master.selectedSensorPosition.toDouble()).toRadians()
    }

    override fun getVelocity(): AngularVelocityMeasureRadiansPerSecond {
        return AngularVelocityMeasureMagEncoderTicksPerHundredMilliseconds(master.selectedSensorVelocity.toDouble()).toRadiansPerSecond()
    }

    override fun setPosition(position: AngularDistanceMeasureRadians) {
        master.selectedSensorPosition = position.toMagEncoderTicks().value.toInt()
    }
}