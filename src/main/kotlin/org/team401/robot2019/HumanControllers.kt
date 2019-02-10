package org.team401.robot2019

import org.snakeskin.dsl.HumanControls
//import org.team401.robot2019.subsystems.drivetrain.Drivetrain
import org.team401.robot2019.subsystems.arm.Arm
import org.snakeskin.logic.Direction
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.subsystems.drivetrain.Drivetrain
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

    whenButton(Buttons.STICK_BOTTOM) {
        pressed {
            Drivetrain.setPose(Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0.0)))
        }
    }
}

val RightStick = HumanControls.t16000m(1) {

}


val Gamepad = HumanControls.dualAction(2){

    whenButton(Buttons.X){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("X button pressed - 1")
        }
    }
    whenButton(Buttons.A){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("A button pressed - 2")
        }
    }
    whenButton(Buttons.B){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("B button pressed - 3")
        }
    }
    whenButton(Buttons.Y){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("Y button pressed - 4")
        }
    }
    whenButton(Buttons.LEFT_BUMPER){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("LB button pressed - 5")
        }
    }
    whenButton(Buttons.RIGHT_BUMPER){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("RB button pressed - 6")
        }
    }
    whenButton(Buttons.LEFT_TRIGGER){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("LT button pressed - 7")
        }
    }
    whenButton(Buttons.RIGHT_TRIGGER){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("RT button pressed - 8")
        }
    }
    whenButton(Buttons.BACK){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("Back button pressed - 9")
        }
    }
    whenButton(Buttons.START){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("Start button pressed - 10")
        }
    }
    whenButton(Buttons.LEFT_STICK){
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("LS button pressed - 11")
        }
    }
    whenButton(Buttons.RIGHT_STICK) {
        pressed {
            //Arm.armMachine.setState(Arm.ArmStates.E_STOPPED)
            println("RS button pressed - 12")
        }
    }
    /*
    whenButton(Buttons.Y){
        pressed {
            println("Y button pressed")
            Arm.armMachine.setState(Arm.ArmStates.MANUAL_CONTROL)
        }
    }
    whenButton(Buttons.RIGHT_BUMPER){
        pressed {
            // Switch tools
            println("Right bumper pressed")
            Arm.armMachine.setState(Arm.ArmStates.SWITCH_TOOL)
        }
    }
    */


    whenHatChanged(Hats.D_PAD){
        when (it){
            Direction.NORTH ->{Arm.setTargetPosition(ControlParameters.ArmPositions.ROCKET_TOP)}
            Direction.SOUTH ->{Arm.setTargetPosition(ControlParameters.ArmPositions.ROCKET_LOW)}
            Direction.EAST ->{Arm.setTargetPosition(ControlParameters.ArmPositions.LOADING_STATION)}
            Direction.WEST ->{Arm.setTargetPosition(ControlParameters.ArmPositions.CARGO_SHIP)}
        }
    }
}
