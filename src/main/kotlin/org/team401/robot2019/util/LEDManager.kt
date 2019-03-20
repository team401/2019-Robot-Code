package org.team401.robot2019.util

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.I2C
import org.lightlink.LightLink
import org.snakeskin.dsl.on
import org.snakeskin.event.Events
import org.snakeskin.logic.History

object LEDManager {
    /**
     * LightLink instance
     */
    private val ll = LightLink(I2C.Port.kMXP)

    /**
     * LightLink indices of the various LED strips
     */
    object Indices {
        const val ArmStrip = 0
        const val TrussBackLeftStrip = 2
        const val TrussFrontLeftStrip = 4
        const val TrussBackRightStrip = 1
        const val TrussFrontRightStrip = 3
    }

    /**
     * Represents the various states for the truss LEDs as a whole
     */
    enum class TrussLedMode {
        Off, //All truss LEDs are off
        Rainbow, //All truss LEDs are in rainbow pattern
        SideIndicator, //Front truss LEDs indicate "red side", back truss LEDs indicate "blue side"
        ModifierRed, //Front truss LEDs are off, back truss LEDs indicate "blue side"
        ModifierBlue //Front truss LEDs indicate "red side", back truss LEDs are off
    }

    /**
     * Represents the various signals for the truss LEDs
     */
    enum class TrussLedSignal {
        FrontTargeted, //Front truss LEDs blink yellow to indicate target found
        BackTargeted, //Back truss LEDs blink yellow to indicate target found
    }

    /**
     * Represents the various states for the LED strip running on the arm
     */
    enum class ArmLedMode {
        Off, //Arm LEDs off
        Rainbow, //Arm LEDs are in rainbow pattern
        HasCargo, //Arm LEDs are orange
        HasHatch, //Arn LEDs are yellow
    }

    private var trussModeHistory = History<TrussLedMode>()
    private var armModeHistory = History<ArmLedMode>()

    /**
     * Sets the truss LEDs to the specified mode
     */
    @Synchronized fun setTrussLedMode(mode: TrussLedMode) {
        when (mode) {
            TrussLedMode.Off -> {
                ll.off(Indices.TrussBackLeftStrip)
                ll.off(Indices.TrussFrontLeftStrip)
                ll.off(Indices.TrussBackRightStrip)
                ll.off(Indices.TrussFrontRightStrip)
            }
            
            TrussLedMode.Rainbow -> {
                ll.rainbow(Indices.TrussBackLeftStrip)
                ll.rainbow(Indices.TrussFrontLeftStrip)
                ll.rainbow(Indices.TrussBackRightStrip)
                ll.rainbow(Indices.TrussFrontRightStrip)
            }
            
            TrussLedMode.SideIndicator -> {
                ll.solid(LightLink.Color.RED, Indices.TrussFrontLeftStrip)
                ll.solid(LightLink.Color.RED, Indices.TrussFrontRightStrip)
                ll.solid(LightLink.Color.BLUE, Indices.TrussBackLeftStrip)
                ll.solid(LightLink.Color.BLUE, Indices.TrussBackRightStrip)
            }

            TrussLedMode.ModifierRed -> {
                ll.off(Indices.TrussFrontLeftStrip)
                ll.off(Indices.TrussFrontRightStrip)
                ll.solid(LightLink.Color.BLUE, Indices.TrussBackLeftStrip)
                ll.solid(LightLink.Color.BLUE, Indices.TrussBackRightStrip)
            }

            TrussLedMode.ModifierBlue -> {
                ll.solid(LightLink.Color.RED, Indices.TrussFrontLeftStrip)
                ll.solid(LightLink.Color.RED, Indices.TrussFrontRightStrip)
                ll.off(Indices.TrussBackLeftStrip)
                ll.off(Indices.TrussBackRightStrip)
            }
        }
        trussModeHistory.update(mode)
    }

    /**
     * Reverts the truss LEDs to their last state
     */
    @Synchronized fun revertTrussLedMode() {
        setTrussLedMode(trussModeHistory.last!!)
    }

    /**
     * Commands the specified signal on the truss LEDs
     */
    fun signalTruss(signal: TrussLedSignal) {
        when (signal) {
            TrussLedSignal.BackTargeted -> {
                ll.signal(LightLink.Color.YELLOW, Indices.TrussBackLeftStrip)
                ll.signal(LightLink.Color.YELLOW, Indices.TrussBackRightStrip)
            }

            TrussLedSignal.FrontTargeted -> {
                ll.signal(LightLink.Color.YELLOW, Indices.TrussFrontLeftStrip)
                ll.signal(LightLink.Color.YELLOW, Indices.TrussFrontRightStrip)
            }
        }
    }

    /**
     * Sets the Arm LEDs to the specified mode
     */
    @Synchronized fun setArmLedMode(mode: ArmLedMode) {
        when (mode) {
            ArmLedMode.Off -> {
                ll.off(Indices.ArmStrip)
            }

            ArmLedMode.Rainbow -> {
                ll.rainbow(Indices.ArmStrip)
            }

            ArmLedMode.HasHatch -> {
                ll.solid(LightLink.Color.YELLOW, Indices.ArmStrip)
            }

            ArmLedMode.HasCargo -> {
                ll.solid(LightLink.Color.ORANGE, Indices.ArmStrip)
            }
        }
        armModeHistory.update(mode)
    }

    /**
     * Reverts the arm LEDs to their last state
     */
    @Synchronized fun revertArmLedMode() {
        setArmLedMode(armModeHistory.last!!)
    }

    private var hatchHistory = History<Boolean>()
    private var cargoHistory = History<Boolean>()

    /**
     * Updates the manager with the currently loaded gamepieces, which can be used to
     * automatically set the arm strip with the appropriate colors
     */
    fun updateGamepieceStatus(hasHatch: Boolean, hasCargo: Boolean) {
        if (DriverStation.getInstance().isOperatorControl) { //Only run this in teleop mode
            hatchHistory.update(hasHatch)
            cargoHistory.update(hasCargo)

            if (hatchHistory.current == true && hatchHistory.last != true) {
                //We got a hatch
                setArmLedMode(ArmLedMode.HasHatch)
            } else if (hatchHistory.current == false && hatchHistory.last == true) {
                //We lost a hatch
                setArmLedMode(ArmLedMode.Off)
            }

            if (cargoHistory.current == true && cargoHistory.last != true) {
                //We got a cargo
                setArmLedMode(ArmLedMode.HasCargo)
            } else if (cargoHistory.current == false && hatchHistory.last == true) {
                //We lost a cargo
                setArmLedMode(ArmLedMode.Off)
            }
        }
    }

    /**
     * Sets all LEDs to off, register event listeners
     */
    fun init() {
        trussModeHistory.update(TrussLedMode.Off) //Push Off into both slots of the histories
        trussModeHistory.update(TrussLedMode.Off)
        armModeHistory.update(ArmLedMode.Off)
        armModeHistory.update(ArmLedMode.Off)

        setTrussLedMode(TrussLedMode.Off)
        setArmLedMode(ArmLedMode.Off)

        on (Events.DISABLED) {
            setTrussLedMode(TrussLedMode.Rainbow)
            setArmLedMode(ArmLedMode.Rainbow)
        }

        on (Events.AUTO_ENABLED) {
            setTrussLedMode(TrussLedMode.Off)
            setArmLedMode(ArmLedMode.Off)
        }

        on (Events.TELEOP_ENABLED) {
            setTrussLedMode(TrussLedMode.SideIndicator)
            setArmLedMode(ArmLedMode.Off)
        }
    }
}