package org.team401.robot2019.util

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.I2C
import org.lightlink.LightLink
import org.snakeskin.dsl.on
import org.snakeskin.event.Events
import org.snakeskin.logic.History
import org.team401.robot2019.RobotEvents
import org.team401.robot2019.control.superstructure.SuperstructureController
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

object LEDManager {
    /**
     * LightLink instance
     */
    private val ll = LightLink(I2C.Port.kMXP)

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
        //SideIndicator, //Front truss LEDs indicate "red side", back truss LEDs indicate "blue side"
        BlueSideActiveLock, //Front truss LEDs are off, back truss LEDs indicate "blue side"
        RedSideActiveLock, //Front truss LEDs indicate "red side", back truss LEDs are off
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

    /**
     * Represents the various states for the LED strip running on the arm
     */
    enum class ArmLedMode {
        Off, //Arm LEDs off
        Rainbow, //Arm LEDs are in rainbow pattern
        HasCargo, //Arm LEDs are orange
        HasHatch, //Arn LEDs are yellow
        Intaking,
        Scoring,
        Climb,
    }

    enum class ToolIndicatorMode {
        Off,
        Rainbow,
        Cargo,
        Hatch
    }

    private var trussModeHistory = History<TrussLedMode>()
    private var armModeHistory = History<ArmLedMode>()
    private var toolIndicatorModeHistory = History<ToolIndicatorMode>()

    /**
     * Sets the truss LEDs to the specified mode
     */
    @Synchronized fun setTrussLedMode(mode: TrussLedMode) {
        if (trussModeHistory.current == mode) return

        when (mode) {
            TrussLedMode.Off -> {
                ll.off(Indices.TrussBackLeftStrip)
                ll.off(Indices.TrussFrontLeftStrip)
                ll.off(Indices.TrussBackRightStrip)
                ll.off(Indices.TrussFrontRightStrip)
            }
            
            TrussLedMode.Rainbow -> {
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussFrontLeftStrip)
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussFrontRightStrip)
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussBackLeftStrip)
                ll.rainbow(LightLink.Speed.SLOW, Indices.TrussBackRightStrip)
            }

            TrussLedMode.Race -> {
                ll.bounce(LightLink.Color.ORANGE, LightLink.Speed.SLOW, Indices.TrussFrontLeftStrip)
                ll.bounce(LightLink.Color.ORANGE, LightLink.Speed.SLOW, Indices.TrussFrontRightStrip)
                ll.bounce(LightLink.Color.ORANGE, LightLink.Speed.SLOW, Indices.TrussBackLeftStrip)
                ll.bounce(LightLink.Color.ORANGE, LightLink.Speed.SLOW, Indices.TrussBackRightStrip)
            }

            /*
            TrussLedMode.SideIndicator -> {
                ll.solid(LightLink.Color.RED, Indices.TrussFrontLeftStrip)
                ll.solid(LightLink.Color.RED, Indices.TrussFrontRightStrip)
                ll.solid(LightLink.Color.BLUE, Indices.TrussBackLeftStrip)
                ll.solid(LightLink.Color.BLUE, Indices.TrussBackRightStrip)
            }
            */

            TrussLedMode.BlueSideActiveLock -> {
                ll.solid(LightLink.Color.BLUE, Indices.TrussBackLeftStrip)
                ll.solid(LightLink.Color.BLUE, Indices.TrussBackRightStrip)
                ll.off(Indices.TrussFrontLeftStrip)
                ll.off(Indices.TrussFrontRightStrip)
            }

            TrussLedMode.RedSideActiveLock -> {
                ll.solid(LightLink.Color.RED, Indices.TrussFrontLeftStrip)
                ll.solid(LightLink.Color.RED, Indices.TrussFrontRightStrip)
                ll.off(Indices.TrussBackLeftStrip)
                ll.off(Indices.TrussBackRightStrip)
            }

            TrussLedMode.BlueSideActiveAuto -> {
                ll.blink(LightLink.Color.BLUE, LightLink.Speed.FAST, Indices.TrussBackLeftStrip)
                ll.blink(LightLink.Color.BLUE, LightLink.Speed.FAST, Indices.TrussBackRightStrip)
                ll.off(Indices.TrussFrontLeftStrip)
                ll.off(Indices.TrussFrontRightStrip)
            }

            TrussLedMode.RedSideActiveAuto -> {
                ll.blink(LightLink.Color.RED, LightLink.Speed.FAST, Indices.TrussFrontLeftStrip)
                ll.blink(LightLink.Color.RED, LightLink.Speed.FAST, Indices.TrussFrontRightStrip)
                ll.off(Indices.TrussBackLeftStrip)
                ll.off(Indices.TrussBackRightStrip)
            }

            TrussLedMode.Climb -> {
                ll.race(LightLink.Color.ORANGE, LightLink.Speed.SLOW, Indices.TrussBackLeftStrip)
                ll.race(LightLink.Color.ORANGE, LightLink.Speed.SLOW, Indices.TrussBackRightStrip)
                ll.race(LightLink.Color.ORANGE, LightLink.Speed.SLOW, Indices.TrussFrontLeftStrip)
                ll.race(LightLink.Color.ORANGE, LightLink.Speed.SLOW, Indices.TrussFrontRightStrip)
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

    /**
     * Sets the Arm LEDs to the specified mode
     */
    @Synchronized fun setArmLedMode(mode: ArmLedMode) {
        when (mode) {
            ArmLedMode.Off -> {
                ll.off(Indices.ArmStrip)
            }

            ArmLedMode.Rainbow -> {
                ll.rainbow(LightLink.Speed.SLOW, Indices.ArmStrip)
            }

            ArmLedMode.HasHatch -> {
                ll.solid(LightLink.Color.YELLOW, Indices.ArmStrip)
            }

            ArmLedMode.HasCargo -> {
                ll.solid(LightLink.Color.ORANGE, Indices.ArmStrip)
            }

            ArmLedMode.Climb -> {
                ll.blink(LightLink.Color.ORANGE, LightLink.Speed.SLOW, Indices.ArmStrip)
            }

            ArmLedMode.Intaking -> {
                ll.solid(LightLink.Color.GREEN, Indices.ArmStrip)
            }

            ArmLedMode.Scoring -> {
                ll.solid(LightLink.Color.BLUE, Indices.ArmStrip)
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

    @Synchronized fun setToolIndicatorLedMode(mode: ToolIndicatorMode) {
        when (mode) {
            ToolIndicatorMode.Off -> {
                ll.off(Indices.ToolIndicatorStrip)
            }

            ToolIndicatorMode.Rainbow -> {
                ll.rainbow(LightLink.Speed.SLOW, Indices.ToolIndicatorStrip)
            }

            ToolIndicatorMode.Cargo -> {
                ll.solid(LightLink.Color.GREEN, Indices.ToolIndicatorStrip)
            }

            ToolIndicatorMode.Hatch -> {
                ll.solid(LightLink.Color.YELLOW, Indices.ToolIndicatorStrip)
            }
        }

        toolIndicatorModeHistory.update(mode)
    }

    private val toolHistory = History<WristMotionPlanner.Tool>()

    @Synchronized fun updateToolStatus(tool: WristMotionPlanner.Tool, force: Boolean = false) {
        if (force || toolHistory.current != tool) {
            when (tool) {
                WristMotionPlanner.Tool.HatchPanelTool -> setToolIndicatorLedMode(ToolIndicatorMode.Hatch)
                WristMotionPlanner.Tool.CargoTool -> setToolIndicatorLedMode(ToolIndicatorMode.Cargo)
            }
        }
        toolHistory.update(tool)
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
        toolIndicatorModeHistory.update(ToolIndicatorMode.Off)
        toolIndicatorModeHistory.update(ToolIndicatorMode.Off)

        setTrussLedMode(TrussLedMode.Off)
        setArmLedMode(ArmLedMode.Off)

        on (Events.DISABLED) {
            setTrussLedMode(TrussLedMode.Rainbow)
            setArmLedMode(ArmLedMode.Rainbow)
            setToolIndicatorLedMode(ToolIndicatorMode.Off)
        }

        on (Events.AUTO_ENABLED) {
            setTrussLedMode(TrussLedMode.Off)
            setArmLedMode(ArmLedMode.Off)
            updateToolStatus(SuperstructureController.output.wristTool, true)
        }

        on (Events.TELEOP_ENABLED) {
            if (SuperstructureRoutines.side == SuperstructureRoutines.Side.FRONT) {
                setTrussLedMode(TrussLedMode.BlueSideActiveLock)
            } else {
                setTrussLedMode(TrussLedMode.RedSideActiveLock)
            }
            setArmLedMode(ArmLedMode.Off)
            updateToolStatus(SuperstructureController.output.wristTool, true)
        }

        on (RobotEvents.VLoc) {
            signalTruss(TrussLedSignal.VLoc)
        }
    }
}