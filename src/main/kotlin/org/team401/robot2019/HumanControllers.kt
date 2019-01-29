package org.team401.robot2019

import org.snakeskin.dsl.HumanControls
//import org.team401.robot2019.subsystems.drivetrain.Drivetrain
import org.team401.robot2019.subsystems.arm.PrototypeArm
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
            //PrototypeArm.armMachine.setState(PrototypeArm.ArmStates.E_STOPPED)
        }
    }
    whenButton(Buttons.Y){
        pressed {
            PrototypeArm.armMachine.setState(PrototypeArm.ArmStates.MANUAL_CONTROL)
        }
    }
}
