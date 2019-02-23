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
    fun enterCC() {
        ArmSubsystem.armPivotMachine.setState(ArmSubsystem.ArmPivotStates.CoordinatedControl)
        ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.CoordinatedControl)
        WristSubsystem.wristMachine.setState(WristSubsystem.WristStates.CoordinatedControl)
    }

    fun goToLow(front: Boolean) {
        enterCC()
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                println("NOT IMPLEMENTED!")
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (front) {
                    SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoBottomFront)
                } else {
                    SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoBottomBack)
                }
            }
        }
    }

    fun goToMid(front: Boolean) {
        enterCC()
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                println("NOT IMPLEMENTED!")
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (front) {
                    SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoMidFront)
                } else {
                    SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoMidBack)
                }        
            }
        }
    }

    fun goToHigh(front: Boolean) {
        enterCC()
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                println("NOT IMPLEMENTED!")
            }

            WristMotionPlanner.Tool.CargoTool -> {
                if (front) {
                    SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoHighFront)
                } else {
                    SuperstructureMotionPlanner.requestMove(ControlParameters.ArmPositions.rocketCargoHighBack)
                }
            }
        }
    }

    fun switchTool() {
        enterCC()
        SuperstructureMotionPlanner.requestToolChange(SuperstructureMotionPlanner.notActiveTool())
    }

    fun intake() {
        enterCC()
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.CargoTool -> {
                //TODO move to intake position
                //TODO turn on cargo wheels
            }

            WristMotionPlanner.Tool.HatchPanelTool -> {
                //TODO move to intake position
                //TODO open hatch panel grabber
            }
        }
    }

    fun stopIntake() {
        enterCC()
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.CargoTool -> {
                //TODO turn off cargo wheels
            }

            WristMotionPlanner.Tool.HatchPanelTool -> {
                //TODO close hatch panel grabber
            }
        }
    }

    fun score(left: Boolean, right: Boolean) {
        enterCC()
        val currentTool = SuperstructureMotionPlanner.activeTool
        when (currentTool) {
            WristMotionPlanner.Tool.HatchPanelTool -> {
                //TODO open claws
            }

            WristMotionPlanner.Tool.CargoTool -> {
                //TODO turn on appropriate wheels
            }
        }
    }
}