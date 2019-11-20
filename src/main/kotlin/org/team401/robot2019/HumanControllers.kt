package org.team401.robot2019

//import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.snakeskin.auto.AutoManager
import org.snakeskin.dsl.HumanControls
import org.snakeskin.logic.Direction
import org.snakeskin.logic.scalars.SquareScalar
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.SuperstructureSideManager
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.vision.LimelightCamera
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.subsystems.*
import org.team401.robot2019.util.LEDManager

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */

val LeftStick = HumanControls.attack3(0) {
    invertAxis(Axes.PITCH)

    deadbandAxis(Axes.PITCH, 0.1)
}

val RightStick = HumanControls.attack3(1) {
    deadbandAxis(Axes.ROLL, 0.1)

    scaleAxis(Axes.ROLL, SquareScalar)
}


val Gamepad = HumanControls.dualAction(2){
    invertAxis(Axes.LEFT_Y)
    whenButton(Buttons.Y) {
        pressed {
            SuperstructureMotionPlanner.goHome()
            SuperstructureRoutines.ccMaybe(true) //Force into coordinated control
            SuperstructureRoutines.sideManager.reportAction(SuperstructureSideManager.Action.SUPERSTRUCTURE_STOWED)
            SuperstructureRoutines.onSideManagerUpdate()
        }
    }

    whenButton(Buttons.X) {
        pressed {
            SuperstructureRoutines.switchTool()
        }
    }

    whenButton(Buttons.B) {
        pressed {
            SuperstructureRoutines.intake()
        }

        released {
            SuperstructureRoutines.stopIntake()
        }
    }

    whenHatChanged(Hats.D_PAD) {
        when (it) {
            Direction.NORTH -> SuperstructureRoutines.goToHigh()
            Direction.WEST -> SuperstructureRoutines.goToMid()
            Direction.SOUTH -> SuperstructureRoutines.goToLow()
            Direction.EAST -> SuperstructureRoutines.goToCargoShip()
        }
    }

    whenButton(Buttons.LEFT_BUMPER) {
        pressed {
            //SuperstructureRoutines.switchSides()
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.Disabled)
        }
        released {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.OpenLoopOperatorControl)
        }
    }

    whenButton(Buttons.RIGHT_BUMPER) {
        pressed {
            SuperstructureRoutines.score()
        }

        released {
            SuperstructureRoutines.stopScoring()
        }
    }

    whenButton(Buttons.LEFT_STICK) {
        pressed {
            SuperstructureRoutines.ccMaybe(true)
            SuperstructureMotionPlanner.setToArmJogMode()
        }
        released {
            SuperstructureMotionPlanner.setToPlanningMode()
        }
    }

    whenButton(Buttons.RIGHT_STICK) {
        pressed {
            SuperstructureRoutines.ccMaybe(true)
            SuperstructureMotionPlanner.setToWristJogMode()
        }
        released {
            SuperstructureMotionPlanner.setToPlanningMode()
        }
    }

    whenButton(Buttons.RIGHT_TRIGGER){
        pressed {
            FloorPickupSubsystem.pickupMachine.setState(FloorPickupSubsystem.PickupStates.Deployed)
            FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Eject)
        }
        released {
            FloorPickupSubsystem.pickupMachine.setState(FloorPickupSubsystem.PickupStates.Stowed)
            FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Idle)
        }
    }
}