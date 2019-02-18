package org.team401.robot2019

import org.snakeskin.dsl.HumanControls
//import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.robot2019.subsystems.ArmSubsystem
import org.snakeskin.logic.Direction
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.robot2019.subsystems.FloorPickupSubsystem
import org.team401.robot2019.subsystems.WristSubsystem
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


val Gamepad = HumanControls.f310(0){
    whenButton(Buttons.A) {
        pressed {
            //WristSubsystem.wristMachine.setState(WristSubsystem.WristStates.GoTo0)
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.GoTo1Point5Foot)
        }
    }

    whenButton(Buttons.X) {
        pressed {
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.GoTo1Foot)
        }
    }

    whenButton(Buttons.Y) {
        pressed {
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.GoTo1Inch)
        }
    }

    whenButton(Buttons.B) {
        pressed {
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.GoTo2Foot)
        }
    }
}
