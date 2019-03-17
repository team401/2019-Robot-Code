package org.team401.robot2019

import org.snakeskin.dsl.HumanControls
//import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.snakeskin.logic.Direction
import org.snakeskin.measure.Inches
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.config.Geometry
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.geometry.Point2d
import org.team401.robot2019.control.superstructure.planning.SuperstructureMotionPlanner
import org.team401.robot2019.control.vision.VisionKinematics
import org.team401.robot2019.control.vision.VisionManager
import org.team401.robot2019.control.vision.VisionOdometryUpdater
import org.team401.robot2019.subsystems.*
import org.team401.robot2019.util.LEDManager
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

    /*
    whenButton(Buttons.STICK_LEFT) {
        pressed {
            //Vision localize
            VisionManager.frontCamera.configForVision(1)
            Thread.sleep(100)

            DrivetrainSubsystem.activeFieldToGoal = VisionKinematics.solveFieldToGoal(
                DrivetrainSubsystem.driveState.getLatestFieldToVehicle().value,
                Geometry.VisionGeometry.robotToFrontCamera,
                VisionManager.frontCamera.frame.toPose2d()
            )
        }
    }

    whenButton(Buttons.STICK_RIGHT) {
        pressed {
            VisionOdometryUpdater.enable(DrivetrainSubsystem.activeFieldToGoal, Geometry.VisionGeometry.robotToFrontCamera)
            DrivetrainSubsystem.driveMachine
        }
    }
*/
    /*
    whenButton(Buttons.STICK_BOTTOM) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.PathFollowing)
        }
        released {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.OpenLoopOperatorControl)
        }
    }
    */
}

val RightStick = HumanControls.t16000m(1) {



    /*
    whenButton(Buttons.STICK_BOTTOM) {
        pressed {
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.TestDown)
        }
        released {
            ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.Disabled)
        }
    }
    */


    whenButton(Buttons.STICK_RIGHT) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbReposition)

            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.Stowed)) {
                //We want to start climbing
                SuperstructureRoutines.ccMaybe(true)
                SuperstructureMotionPlanner.goToClimb()
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.DownL3)
            } else if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.LondonBridgeIsMaybeFallingDown)) {
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.FallL3)
            }

        }
        released {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbStop)
            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.DownL3)) {
                //We are in the process of climbing, slowly go down
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.SlowFall)
            } else if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.FallL3)) {
                //We have climbed up and pulled ourselves onto the platform, we now want to retract the front legs
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.LondonBridgeIsMaybeFallingDown)
            }
        }
    }

    whenButton(Buttons.STICK_LEFT) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbReposition)

            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.Stowed)) {
                //We want to start climbing
                SuperstructureRoutines.ccMaybe(true)
                SuperstructureMotionPlanner.goToClimb()
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.DownL2)
            } else if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.LondonBridgeIsMaybeFallingDown)) {
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.FallL2)
            }

        }
        released {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbStop)
            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.DownL2)) {
                //We are in the process of climbing, slowly go down
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.SlowFall)
            } else if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.FallL2)) {
                //We have climbed up and pulled ourselves onto the platform, we now want to retract the front legs
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.LondonBridgeIsMaybeFallingDown)
            }
        }
    }


}


val Gamepad = HumanControls.dualAction(2){
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
            SuperstructureRoutines.intake(readButton(Buttons.RIGHT_TRIGGER), readButton(Buttons.LEFT_TRIGGER))
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
            Direction.NORTH -> SuperstructureRoutines.goToHigh(readButton(Buttons.RIGHT_TRIGGER), readButton(Buttons.LEFT_TRIGGER))
            Direction.WEST -> SuperstructureRoutines.goToMid(readButton(Buttons.RIGHT_TRIGGER), readButton(Buttons.LEFT_TRIGGER))
            Direction.SOUTH -> SuperstructureRoutines.goToLow(readButton(Buttons.RIGHT_TRIGGER), readButton(Buttons.LEFT_TRIGGER))
            Direction.EAST -> SuperstructureRoutines.goToCargoShip(readButton(Buttons.RIGHT_TRIGGER), readButton(Buttons.LEFT_TRIGGER))
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

    whenButton(Buttons.LEFT_TRIGGER) {
        pressed {
            LEDManager.setTrussLedMode(LEDManager.TrussLedMode.ModifierBlue)
        }
        released {
            LEDManager.setTrussLedMode(LEDManager.TrussLedMode.SideIndicator)
        }
    }

    whenButton(Buttons.RIGHT_TRIGGER) {
        pressed {
            LEDManager.setTrussLedMode(LEDManager.TrussLedMode.ModifierRed)
        }
        released {
            LEDManager.setTrussLedMode(LEDManager.TrussLedMode.SideIndicator)
        }
    }
}