package org.team401.robot2019.control.superstructure

import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.ArmSubsystem
import org.team401.robot2019.subsystems.FloorPickupSubsystem
import org.team401.robot2019.subsystems.WristSubsystem
import org.team401.robot2019.util.LEDManager

/**
 * @author Cameron Earle
 * @version 2/23/2019
 *
 */
object SuperstructureRoutines {
    enum class Side {
        FRONT, BACK
    }

    val sideManager = SuperstructureSideManager()

    var side = Side.FRONT
    @Synchronized get
    @Synchronized set

    /**
     * Call this directly after calling a side manager update function
     */
    @Synchronized
    fun onSideManagerUpdate() {
        val sideManagerSide = sideManager.getSide()
        val sideManagerLocked = sideManager.isLocked()

        side = sideManagerSide

        when (sideManagerSide) {
            Side.FRONT -> {
                if (sideManagerLocked) {
                    //Front locked
                    LEDManager.setTrussLedMode(LEDManager.TrussLedMode.RedSideActiveLock)
                } else {
                    //Front auto
                    LEDManager.setTrussLedMode(LEDManager.TrussLedMode.RedSideActiveAuto)
                }
            }

            Side.BACK -> {
                if (sideManagerLocked) {
                    //Back locked
                    LEDManager.setTrussLedMode(LEDManager.TrussLedMode.BlueSideActiveLock)
                } else {
                    //Back auto
                    LEDManager.setTrussLedMode(LEDManager.TrussLedMode.BlueSideActiveAuto)
                }
            }
        }
    }

    @Synchronized fun switchSides() {
        sideManager.reportAction(SuperstructureSideManager.Action.TOGGLED) //Toggle the side manager
        /*
        if (side == Side.FRONT){
            side = Side.BACK
            LEDManager.setTrussLedMode(LEDManager.TrussLedMode.RedSideActiveLock)
        } else {
            side = Side.FRONT
            LEDManager.setTrussLedMode(LEDManager.TrussLedMode.BlueSideActiveLock)
        }
         */
    }

    fun ccMaybe(enterCC: Boolean): Boolean {
        if (enterCC) {
            ArmSubsystem.armPivotMachine.setState(ArmSubsystem.ArmPivotStates.CoordinatedControl)
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.CoordinatedControl)
            WristSubsystem.wristMachine.setState(WristSubsystem.WristStates.CoordinatedControl)
            //
        } else {
            //ArmSubsystem.armPivotMachine.setState(ArmSubsystem.ArmPivotStates.Holding)
            //ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.Holding)
            //WristSubsystem.wristMachine.setState(WristSubsystem.WristStates.Holding)
        }
        return enterCC
    }

    @Synchronized
    fun goToCargoShip() {
        sideManager.reportAction(SuperstructureSideManager.Action.SUPERSTRUCTURE_MOVED_TO_SETPOINT)
        onSideManagerUpdate()

        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                //not possible
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.cargoShipCargoFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.cargoShipCargoBack))
                }
            }
        }
    }

    @Synchronized
    fun goToLow() {
        sideManager.reportAction(SuperstructureSideManager.Action.SUPERSTRUCTURE_MOVED_TO_SETPOINT)
        onSideManagerUpdate()

        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    if (ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchBottomFront))) {
                        synchronized (this) {

                        }
                    }
                } else if (side == Side.BACK) {
                    if (ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchBottomBack))) {
                        synchronized (this) {
                            sideManager.reportAction(SuperstructureSideManager.Action.SUPERSTRUCTURE_MOVED_TO_SETPOINT)
                            onSideManagerUpdate()
                        }
                    }
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketCargoBottomFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketCargoBottomBack))
                }
            }
        }
    }

    @Synchronized
    fun goToMid() {
        sideManager.reportAction(SuperstructureSideManager.Action.SUPERSTRUCTURE_MOVED_TO_SETPOINT)
        onSideManagerUpdate()

        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchMidFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchMidBack))
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketCargoMidFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketCargoMidBack))
                }        
            }
        }
    }

    @Synchronized
    fun goToHigh() {
        sideManager.reportAction(SuperstructureSideManager.Action.SUPERSTRUCTURE_MOVED_TO_SETPOINT)
        onSideManagerUpdate()

        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchHighFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchHighBack))
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketCargoHighFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketCargoHighBack))
                }
            }
        }
    }

    @Synchronized
    fun switchTool() {
        sideManager.reportAction(SuperstructureSideManager.Action.SUPERSTRUCTURE_MOVED_TO_SETPOINT)
        onSideManagerUpdate()

        ccMaybe(SuperstructureMotionPlanner.requestToolChange(SuperstructureMotionPlanner.notActiveTool()))
    }

    @Synchronized
    fun intake() {
        sideManager.reportAction(SuperstructureSideManager.Action.INTAKE_STARTED)
        onSideManagerUpdate()

        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.CargoTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.cargoFloorPickupFront))
                } else if (side == Side.BACK){
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.cargoFloorPickupBack))
                }
                WristSubsystem.cargoGrabberMachine.setState(WristSubsystem.CargoGrabberStates.Clamped)
                WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Intake)
            }

            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.hatchIntakeFront))
                } else if (side == Side.BACK){
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.hatchIntakeBack))
                }
                WristSubsystem.hatchClawMachine.setState(WristSubsystem.HatchClawStates.Unclamped)
            }
        }
        LEDManager.setArmLedMode(LEDManager.ArmLedMode.Intaking)
    }

    @Synchronized
    fun stopIntake() {
        sideManager.reportAction(SuperstructureSideManager.Action.INTAKE_FINISHED)
        onSideManagerUpdate()

        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.CargoTool -> {
                WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Idle)
                WristSubsystem.cargoGrabberMachine.setState(WristSubsystem.CargoGrabberStates.Clamped)
            }

            WristMotionPlanner.Tool.HatchPanelTool -> {
                WristSubsystem.hatchClawMachine.setState(WristSubsystem.HatchClawStates.Clamped)
            }
        }
        LEDManager.setArmLedMode(LEDManager.ArmLedMode.Off)
    }

    @Synchronized
    fun score() {
        sideManager.reportAction(SuperstructureSideManager.Action.SCORE_STARTED)
        onSideManagerUpdate()

        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                WristSubsystem.hatchClawMachine.setState(WristSubsystem.HatchClawStates.Unclamped)
                WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Idle)
            }

            WristMotionPlanner.Tool.CargoTool -> {
                WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Scoring)
            }
        }
        LEDManager.setArmLedMode(LEDManager.ArmLedMode.Scoring)
    }

    @Synchronized
    fun stopScoring() {
        sideManager.reportAction(SuperstructureSideManager.Action.SCORE_FINISHED)
        onSideManagerUpdate()

        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.CargoTool -> {
                WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Idle)
            }
            WristMotionPlanner.Tool.HatchPanelTool -> {
                WristSubsystem.hatchClawMachine.setState(WristSubsystem.HatchClawStates.Clamped)
            }
        }
        LEDManager.setArmLedMode(LEDManager.ArmLedMode.Off)
    }

    @Synchronized
    fun goToFloorPickup() {
        score()
        SuperstructureMotionPlanner.goToFloorPickup()

        FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Intake)
        Thread.sleep(250)
        FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Idle)
    }

    fun returnFromFloorPickup() {
        stopIntake()
        Thread.sleep(250)
        FloorPickupSubsystem.pickupMachine.setState(FloorPickupSubsystem.PickupStates.Deployed)
        FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Eject)
        Thread.sleep(250)

        SuperstructureMotionPlanner.returnFromFloorPickup()

        Thread.sleep(500)

        FloorPickupSubsystem.pickupMachine.setState(FloorPickupSubsystem.PickupStates.Stowed)
        FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Idle)
    }

    fun intakeCargoFromLoadingStation() {
        //no-op (DO NOT USE!)
        /*
        if (SuperstructureMotionPlanner.activeTool == WristMotionPlanner.Tool.CargoTool) {
            when (side){
                Side.FRONT -> ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.cargoIntakeLoadingStationFront))
                Side.BACK -> ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.cargoIntakeLoadingStationBack))
            }
            WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Intake)
        }
         */
    }
}