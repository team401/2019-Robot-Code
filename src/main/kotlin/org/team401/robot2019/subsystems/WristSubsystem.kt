package org.team401.robot2019.subsystems

import com.ctre.phoenix.motorcontrol.SensorCollection
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import org.snakeskin.component.impl.CTRESmartGearbox
import org.snakeskin.dsl.*
import org.team401.robot2019.control.superstructure.geometry.WristState

/**
 * @author Cameron Earle
 * @version 2/10/2019
 *
 */
object WristSubsystem: Subsystem() {
    private val rotationTalon = TalonSRX(50)
    val leftIntakeTalon = TalonSRX(51)
    val rightIntakeTalon = TalonSRX(52)

    private val rotation = CTRESmartGearbox(rotationTalon)

    private val cargoSensor = DigitalInput(0)

    /**
     * Returns whether or not the sensor system sees a cargo present in the intake.
     * The sensor should be located in the front of the intake such that it can be used
     * to both automatically close the intake as well as detect continuous presence.
     */
    private fun systemSeesCargo(): Boolean {
        return cargoSensor.get() //TODO check polarity of banner sensor
    }

    /**
     * Returns whether or not the sensor system sees a hatch present in the claw.
     * The sensor(s) should be located such that they reliably indicate the presence
     * of a hatch regardless of orientation or position in the claw
     */
    private fun systemSeesHatch(): Boolean {
        //We use the limit switch ports on the rotation talon to read the hatch sensors
        val sc = rotation.getSensorCollection()
        val hasLeft = sc.isRevLimitSwitchClosed //TODO check polarity
        val hasRight = sc.isFwdLimitSwitchClosed

        return hasLeft || hasRight //Use or here so that we fail on the safe side and assume we have a stuck gamepiece
    }

    /**
     * Gets the current state of the wrist as measured by sensors
     */
    fun getCurrentWristState(): WristState {
        val wristAngle = rotation.getPosition()

        return WristState(wristAngle, systemSeesCargo(), systemSeesHatch())
    }
}