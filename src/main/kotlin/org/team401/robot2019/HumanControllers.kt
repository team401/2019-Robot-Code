package org.team401.robot2019

import org.snakeskin.dsl.HumanControls
//import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.robot2019.subsystems.ArmSubsystem
import org.snakeskin.logic.Direction
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.robot2019.subsystems.FloorPickupSubsystem
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
            DrivetrainSubsystem.setPose(Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0.0)))
        }
    }
}

val RightStick = HumanControls.t16000m(1) {

    whenButton(Buttons.STICK_RIGHT) {
        pressed {
            FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Intake)
        }
        released {
            FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Idle)
        }
    }

    whenButton(Buttons.STICK_LEFT) {
        pressed {
            FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Eject)
        }
        released {
            FloorPickupSubsystem.wheelsMachine.setState(FloorPickupSubsystem.WheelsStates.Idle)
        }
    }
}


val Gamepad = HumanControls.dualAction(2){

    whenButton(Buttons.X){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("X button pressed - 1")
        }
    }
    whenButton(Buttons.A){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("A button pressed - 2")
        }
    }
    whenButton(Buttons.B){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("B button pressed - 3")
        }
    }
    whenButton(Buttons.Y){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("Y button pressed - 4")
        }
    }
    whenButton(Buttons.LEFT_BUMPER){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("LB button pressed - 5")
        }
    }
    whenButton(Buttons.RIGHT_BUMPER){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("RB button pressed - 6")
        }
    }
    whenButton(Buttons.LEFT_TRIGGER){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("LT button pressed - 7")
        }
    }
    whenButton(Buttons.RIGHT_TRIGGER){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("RT button pressed - 8")
        }
    }
    whenButton(Buttons.BACK){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("Back button pressed - 9")
        }
    }
    whenButton(Buttons.START){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("Start button pressed - 10")
        }
    }
    whenButton(Buttons.LEFT_STICK){
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("LS button pressed - 11")
        }
    }
    whenButton(Buttons.RIGHT_STICK) {
        pressed {
            //ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.E_STOPPED)
            println("RS button pressed - 12")
        }
    }
    /*
    whenButton(Buttons.Y){
        pressed {
            println("Y button pressed")
            ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.MANUAL_CONTROL)
        }
    }
    whenButton(Buttons.RIGHT_BUMPER){
        pressed {
            // Switch tools
            println("Right bumper pressed")
            ArmSubsystem.armMachine.setState(ArmSubsystem.ArmStates.SWITCH_TOOL)
        }
    }
    */


    whenHatChanged(Hats.D_PAD){
        when (it){
            Direction.NORTH ->{
                ArmSubsystem.setTargetPosition(ControlParameters.ArmPositions.ROCKET_TOP)}
            Direction.SOUTH ->{
                ArmSubsystem.setTargetPosition(ControlParameters.ArmPositions.ROCKET_LOW)}
            Direction.EAST ->{
                ArmSubsystem.setTargetPosition(ControlParameters.ArmPositions.LOADING_STATION)}
            Direction.WEST ->{
                ArmSubsystem.setTargetPosition(ControlParameters.ArmPositions.CARGO_SHIP)}
        }
    }
}
