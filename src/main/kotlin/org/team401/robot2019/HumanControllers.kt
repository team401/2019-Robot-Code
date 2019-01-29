package org.team401.robot2019

import org.snakeskin.dsl.HumanControls
//import org.team401.robot2019.subsystems.drivetrain.Drivetrain
import org.team401.robot2019.subsystems.arm.Arm
import org.snakeskin.logic.Direction
import org.team401.robot2019.config.ControlParameters

/**
 * @author Cameron Earle
 * @version 1/5/2019
 *
 */
val LeftStick = HumanControls.t16000m(0) {
    invertAxis(Axes.PITCH)

    /*
    whenButton(Buttons.STICK_BOTTOM) {
        pressed {
            Drivetrain.setPose(Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0.0)))
        }
    }

    whenButton(Buttons.TRIGGER) {
        pressed {
            Drivetrain.driveMachine.setState(Drivetrain.DriveStates.PathFollowing)
        }
        released {
            Drivetrain.driveMachine.setState(Drivetrain.DriveStates.DriverControl)
        }
    }
    */
}

val RightStick = HumanControls.t16000m(1) {

}
val Gamepad = HumanControls.f310(2){

    whenButton(Buttons.B){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
        }
    }
    whenButton(Buttons.Y){
        pressed {
            Arm.armMachine.setState(Arm.ArmStates.MANUAL_CONTROL)
        }
    }
    whenHatChanged(Hats.D_PAD){
        when (it){
            Direction.NORTH ->{Arm.setTargetPosition(ControlParameters.ArmPositions.ROCKET_TOP)}
            Direction.SOUTH ->{Arm.setTargetPosition(ControlParameters.ArmPositions.ROCKET_LOW)}
            Direction.EAST ->{Arm.setTargetPosition(ControlParameters.ArmPositions.LOADING_STATION)}
            Direction.WEST ->{Arm.setTargetPosition(ControlParameters.ArmPositions.CARGO_SHIP)}
        }
    }
}
