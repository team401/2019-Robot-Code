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

    var side = Side.FRONT
    @Synchronized get
    @Synchronized set

    @Synchronized fun switchSides() {
        if (side == Side.FRONT){
            side = Side.BACK
            LEDManager.setTrussLedMode(LEDManager.TrussLedMode.ModifierBlue)
        } else {
            side = Side.FRONT
            LEDManager.setTrussLedMode(LEDManager.TrussLedMode.ModifierRed)
        }
    }

    fun ccMaybe(enterCC: Boolean) {
        if (enterCC) {
            ArmSubsystem.armPivotMachine.setState(ArmSubsystem.ArmPivotStates.CoordinatedControl)
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.CoordinatedControl)
            WristSubsystem.wristMachine.setState(WristSubsystem.WristStates.CoordinatedControl)
        } else {
            //ArmSubsystem.armPivotMachine.setState(ArmSubsystem.ArmPivotStates.Holding)
            //ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.Holding)
            //WristSubsystem.wristMachine.setState(WristSubsystem.WristStates.Holding)
        }
    }

    fun goToCargoShip() {
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

    fun goToLow() {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchBottomFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchBottomBack))
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

    fun goToMid() {
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

    fun goToHigh() {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchHighFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketHatchHighBack))
                }            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketCargoHighFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.rocketCargoHighBack))
                }
            }
        }
    }

    fun switchTool() {
        ccMaybe(SuperstructureMotionPlanner.requestToolChange(SuperstructureMotionPlanner.notActiveTool()))
    }

    fun intake() {
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

    fun stopIntake() {
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

    fun score() {
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

    fun stopScoring() {
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

        FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Eject)
        Thread.sleep(250)

        SuperstructureMotionPlanner.returnFromFloorPickup()

        FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Idle)
    }

    fun intakeCargoFromLoadingStation() {
        if (SuperstructureMotionPlanner.activeTool == WristMotionPlanner.Tool.CargoTool) {
            when (side){
                Side.FRONT -> ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.cargoIntakeLoadingStationFront))
                Side.BACK -> ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.SuperstructurePositions.cargoIntakeLoadingStationBack))
            }
            WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Intake)
        }
    }
}