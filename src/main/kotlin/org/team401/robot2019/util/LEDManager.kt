package org.team401.robot2019.util

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.I2C
import org.lightlink.LightLink
import org.snakeskin.dsl.on
import org.snakeskin.event.Events
import org.snakeskin.logic.History
import org.team401.lightlink.LightLinkPatched
import org.team401.robot2019.RobotEvents
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

object LEDManager {
    /**
     * LightLink instance
     */
    private val ll = LightLinkPatched(I2C.Port.kMXP)

    /**
     * LightLink indices of the various LED strips
     */
    object Indices {
        const val ArmStrip = 4
        const val TrussBackLeftStrip = 0
        const val TrussFrontLeftStrip = 1
        const val TrussBackRightStrip = 2
        const val TrussFrontRightStrip = 3
        const val ToolIndicatorStrip = 5
    }

    /**
     * Represents the various states for the truss LEDs as a whole
     */
    enum class TrussLedMode {
        Off, //All truss LEDs are off
        Rainbow, //All truss LEDs are in rainbow pattern
        Race, //All truss LEDs race orange for maximum robot performance
        BlueSideActiveLock, //Front truss LEDs are off, back truss LEDs indicate "blue side"
        BlueSideActiveLockVision,
        RedSideActiveLock, //Front truss LEDs indicate "red side", back truss LEDs are off
        RedSideActiveLockVision,
        CargoLock,
        BlueSideActiveAuto,
        RedSideActiveAuto,
        Climb,
    }

    /**
     * Represents the various signals for the truss LEDs
     */
    enum class TrussLedSignal {
        FrontTargeted, //Front truss LEDs blink yellow to indicate target found
        BackTargeted, //Back truss LEDs blink yellow to indicate target found
        WristHomed, //The wrist is homed
        VLoc, //Vision localized
    }

    enum class ArmLedSignal {
        HatchAcquired
    }

    /**
     * Represents the various states for the LED strip running on the arm
     */
    enum class ArmLedMode {
        Off, //Arm LEDs off
        Rainbow, //Arm LEDs are in rainbow pattern
        Intaking,
        Scoring,
        Climb,
    }

    private var trussModeHistory = History<TrussLedMode>()
    private var armModeHistory = History<ArmLedMode>()

    /**
     * Sets the truss LEDs to the specified mode
     */
    @Synchronized fun setTrussLedMode(mode: TrussLedMode) {
        if (trussModeHistory.current == mode) return

        when (mode) {
            TrussLedMode.Off -> {
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussFrontLeftStrip)
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussFrontRightStrip)
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussBackLeftStrip)
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussBackRightStrip)
            }
            
            TrussLedMode.Rainbow -> {
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussFrontLeftStrip)
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussFrontRightStrip)
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussBackLeftStrip)
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussBackRightStrip)
            }

            TrussLedMode.Race -> {

            }

            TrussLedMode.BlueSideActiveLock -> {

            }

            TrussLedMode.RedSideActiveLock -> {

            }

            TrussLedMode.BlueSideActiveAuto -> {

            }

            TrussLedMode.RedSideActiveAuto -> {

            }

            TrussLedMode.BlueSideActiveLockVision -> {

            }

            TrussLedMode.RedSideActiveLockVision -> {

            }

            TrussLedMode.CargoLock -> {

            }

            TrussLedMode.Climb -> {

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

            TrussLedSignal.WristHomed -> {
                ll.signal(LightLink.Color.GREEN, Indices.TrussBackLeftStrip)
                ll.signal(LightLink.Color.GREEN, Indices.TrussBackRightStrip)
                ll.signal(LightLink.Color.GREEN, Indices.TrussFrontLeftStrip)
                ll.signal(LightLink.Color.GREEN, Indices.TrussFrontRightStrip)
            }

            TrussLedSignal.VLoc -> {
                ll.signal(LightLink.Color.YELLOW, Indices.TrussBackLeftStrip)
                ll.signal(LightLink.Color.YELLOW, Indices.TrussBackRightStrip)
                ll.signal(LightLink.Color.YELLOW, Indices.TrussFrontLeftStrip)
                ll.signal(LightLink.Color.YELLOW, Indices.TrussFrontRightStrip)
            }
        }
    }

    fun signalArm(signal: ArmLedSignal) {
        when (signal) {
            ArmLedSignal.HatchAcquired -> {
                ll.signal(LightLink.Color.GREEN, Indices.ArmStrip)
            }
        }
    }

    /**
     * Sets the Arm LEDs to the specified mode
     */
    @Synchronized fun setArmLedMode(mode: ArmLedMode) {
        if (armModeHistory.current == mode) return

        when (mode) {
            ArmLedMode.Off -> {
                ll.rainbow(LightLink.Speed.SLOW, Indices.ArmStrip)
            }

            ArmLedMode.Rainbow -> {
                ll.rainbow(LightLink.Speed.SLOW, Indices.ArmStrip)
            }

            ArmLedMode.Climb -> {
            }

            ArmLedMode.Intaking -> {
            }

            ArmLedMode.Scoring -> {
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

        on (RobotEvents.HatchAcquired) {
            signalArm(ArmLedSignal.HatchAcquired)
        }

        on (Events.TELEOP_ENABLED) {
            if (SuperstructureRoutines.side == SuperstructureRoutines.Side.FRONT) {
                setTrussLedMode(TrussLedMode.BlueSideActiveLock)
            } else {
                setTrussLedMode(TrussLedMode.RedSideActiveLock)
            }
            setArmLedMode(ArmLedMode.Off)
        }
    }
}