package org.team401.robot2019.control.superstructure

import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.ArmSubsystem
import org.team401.robot2019.subsystems.WristSubsystem

/**
 * @author Cameron Earle
 * @version 2/23/2019
 *
 */
object SuperstructureRoutines {
    fun ccMaybe(enterCC: Boolean) {
        if (enterCC) {
            ArmSubsystem.armPivotMachine.setState(ArmSubsystem.ArmPivotStates.CoordinatedControl)
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.CoordinatedControl)
            WristSubsystem.wristMachine.setState(WristSubsystem.WristStates.CoordinatedControl)
        } else {
            ArmSubsystem.armPivotMachine.setState(ArmSubsystem.ArmPivotStates.Holding)
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.Holding)
            WristSubsystem.wristMachine.setState(WristSubsystem.WristStates.Holding)
        }
    }

    fun goToLow(front: Boolean, back: Boolean) {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (front && !back) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchBottomFront))
                } else if (back && !front) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchBottomBack))
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (front && !back) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoBottomFront))
                } else if (back && !front) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoBottomBack))
                }
            }
        }
    }

    fun goToMid(front: Boolean, back: Boolean) {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (front && !back) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchMidFront))
                } else if (back && !front) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchMidBack))
                }
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (front && !back) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoMidFront))
                } else if (back && !front) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoMidBack))
                }        
            }
        }
    }

    fun goToHigh(front: Boolean, back: Boolean) {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (front && !back) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchHighFront))
                } else if (back && !front) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketHatchHighBack))
                }            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (front && !back) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoHighFront))
                } else if (back && !front) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoHighBack))
                }
            }
        }
    }

    fun switchTool() {
        ccMaybe(SuperstructureMotionPlanner.requestToolChange(SuperstructureMotionPlanner.notActiveTool()))
    }

    fun intake(front: Boolean, back: Boolean) {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.CargoTool -> {
                if (front && !back) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.cargoFloorPickupFront))
                } else if (back && !front){
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.cargoFloorPickupBack))
                }
                if (back != front) { //If they only pushed one button, this is effectively making sure this doesn't run if neither case above ran
                    WristSubsystem.cargoGrabberMachine.setState(WristSubsystem.CargoGrabberStates.Unclamped)
                    WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Intake)
                }
            }

            WristMotionPlanner.Tool.HatchPanelTool -> {
                if (front && !back) {
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.hatchIntakeFront))
                } else if (back && !front){
                    ccMaybe(SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.hatchIntakeBack))
                }
                if (back != front) { //If they only pushed one button, this is effectively making sure this doesn't run if neither case above ran
                    WristSubsystem.hatchClawMachine.setState(WristSubsystem.HatchClawStates.Unclamped)
                }
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

    fun scoreLeft() {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                WristSubsystem.hatchClawMachine.setState(WristSubsystem.HatchClawStates.Unclamped)
            }

            WristMotionPlanner.Tool.CargoTool -> {
                //TODO turn on appropriate wheels
            }
        }
    }

    fun scoreRight() {
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                WristSubsystem.hatchClawMachine.setState(WristSubsystem.HatchClawStates.Unclamped)
            }

            WristMotionPlanner.Tool.CargoTool -> {
                //TODO turn on appropriate wheels
            }
        }
    }
}