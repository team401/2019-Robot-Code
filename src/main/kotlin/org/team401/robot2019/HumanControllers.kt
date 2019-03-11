package org.team401.robot2019

import org.snakeskin.dsl.HumanControls
//import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.snakeskin.logic.Direction
import org.snakeskin.measure.Inches
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.subsystems.*
import org.team401.taxis.geometry.Pose2d
import org.team401.taxis.geometry.Rotation2d
import org.team401.taxis.geometry.Translation2d

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */

val LeftStick = HumanControls.t16000m(0) {
    invertAxis(Axes.PITCH)

    whenButton(Buttons.TRIGGER) {
        pressed {
            DrivetrainSubsystem.shift(DrivetrainSubsystem.ShifterStates.LOW)
        }
        released {
            DrivetrainSubsystem.shift(DrivetrainSubsystem.ShifterStates.HIGH)
        }
    }

    whenButton(Buttons.STICK_BOTTOM) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.PathFollowing)
        }
        released {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.OpenLoopOperatorControl)
        }
    }
}

val RightStick = HumanControls.t16000m(1) {

}

val Gamepad = HumanControls.f310(2){
    whenButton(Buttons.Y) {
        pressed {
            SuperstructureMotionPlanner.goHome()
            SuperstructureRoutines.ccMaybe(true) //Force into coordinated control
        }
    }

    whenButton(Buttons.X) {
        pressed {
            SuperstructureRoutines.switchTool()
        }
    }

    //TODO floor pickup is button A

    whenButton(Buttons.B) {
        pressed {
            SuperstructureRoutines.intake(readAxis(Axes.LEFT_TRIGGER) > 0.5, readAxis(Axes.RIGHT_TRIGGER) > 0.5)
        }

        released {
            SuperstructureRoutines.stopIntake()
        }
    }

    whenButton(Buttons.RIGHT_BUMPER) {
        pressed {
            WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Scoring)
        }
        released {
            WristSubsystem.cargoWheelsMachine.setState(WristSubsystem.CargoWheelsStates.Idle)
        }
    }

    whenHatChanged(Hats.D_PAD) {
        when (it) {
            Direction.NORTH -> SuperstructureRoutines.goToHigh(readAxis(Axes.LEFT_TRIGGER) > 0.5, readAxis(Axes.RIGHT_TRIGGER) > 0.5)
            Direction.WEST -> SuperstructureRoutines.goToMid(readAxis(Axes.LEFT_TRIGGER) > 0.5, readAxis(Axes.RIGHT_TRIGGER) > 0.5)
            Direction.SOUTH -> SuperstructureRoutines.goToLow(readAxis(Axes.LEFT_TRIGGER) > 0.5, readAxis(Axes.RIGHT_TRIGGER) > 0.5)
            //TODO add cargo ship
        }
    }

    whenButton(Buttons.LEFT_BUMPER) {
        pressed {
            SuperstructureRoutines.score()
        }

        released {
            SuperstructureRoutines.stopScoring(false, false)
        }
    }

    whenButton(Buttons.RIGHT_BUMPER) {
        pressed {
            SuperstructureRoutines.score()
        }

        released {
            SuperstructureRoutines.stopScoring(false, false)
        }
    }
}

/*
val LeftStick = HumanControls.attack3(0) {
    invertAxis(Axes.PITCH)

    whenButton(Buttons.STICK_TOP) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbAlign)
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.DownL3)
        }
    }

    whenButton(Buttons.STICK_BOTTOM) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.OpenLoopOperatorControl)
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.Stowed)
        }
    }
}

val RightStick = HumanControls.attack3(1) {
    whenButton(Buttons.STICK_TOP) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbAlign)
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.FallL3)
        }
    }

    whenButton(Buttons.STICK_BOTTOM) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbAlign)
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.Stowed)
        }
    }
}

/*
val TestGamepad = HumanControls.f310(2) {
    whenButton(Buttons.Y) {
        pressed {
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.DownL3)
        }
    }

    whenButton(Buttons.A) {
        pressed {
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.Stowed)
        }
    }

    whenButton(Buttons.B) {
        pressed {
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.Disabled)
        }
    }

    whenButton(Buttons.X) {
        pressed {
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.FallL3)
        }
    }
}
        */