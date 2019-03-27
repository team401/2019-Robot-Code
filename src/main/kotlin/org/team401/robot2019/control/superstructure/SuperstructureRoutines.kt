package org.team401.robot2019.control.superstructure

import org.snakeskin.logic.LockingDelegate
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.control.superstructure.planning.command.ExtensionOnlyCommand
import org.team401.robot2019.subsystems.ArmSubsystem
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
    @Synchronized private set

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
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.cargoShipCargoFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.cargoShipCargoBack))
                }
            }
        }
    }

    fun goToLow() {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchBottomFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchBottomBack))
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoBottomFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoBottomBack))
                }
            }
        }
    }

    fun goToMid() {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchMidFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchMidBack))
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoMidFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoMidBack))
                }        
            }
        }
    }

    fun goToHigh() {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchHighFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchHighBack))
                }            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoHighFront))
                } else if (side == Side.BACK) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoHighBack))
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
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.cargoFloorPickupFront))
                } else if (side == Side.BACK){
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.cargoFloorPickupBack))
                }
                WristSubsystem.cargoGrabberMachine.setState(WristSubsystem.CargoGrabberStates.Clamped)
                WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Intake)
            }

            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (side == Side.FRONT) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.hatchIntakeFront))
                } else if (side == Side.BACK){
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.hatchIntakeBack))
                }
                WristSubsystem.hatchClawMachine.setState(WristSubsystem.HatchClawStates.Unclamped)
            }
        }
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
    }

    fun stopScoring() {
        WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Idle)
    }




}