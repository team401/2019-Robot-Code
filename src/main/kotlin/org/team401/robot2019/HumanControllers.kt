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
            SuperstructureRoutines.enterCC()
            SuperstructureMotionPlanner.goHome()
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
            SuperstructureRoutines.intake()
        }

        released {
            SuperstructureRoutines.stopIntake()
        }
    }

    whenHatChanged(Hats.D_PAD) {
        when (it) {
            Direction.NORTH -> SuperstructureRoutines.goToHigh(true) //TODO add in accessor
            Direction.WEST -> SuperstructureRoutines.goToMid(true)
            Direction.SOUTH -> SuperstructureRoutines.goToLow(true)
            //TODO add cargo ship
        }
    }
}
