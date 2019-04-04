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
import org.team401.robot2019.control.vision.LimelightCamera
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

    whenButton(Buttons.STICK_BOTTOM) {
        pressed {
            //Vision aligner
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.VisionAlign)
        }

        released {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.OpenLoopOperatorControl)
        }
    }

    whenButton(Buttons.STICK_LEFT) {
        pressed {
            WristSubsystem.wristMachine.setState(WristSubsystem.WristStates.ForceTo0)
            Thread.sleep(1000)
            ArmSubsystem.armExtensionMachine.setState(ArmSubsystem.ArmExtensionStates.Homing)
        }
    }

    whenButton(Buttons.STICK_RIGHT) {
        pressed {
            VisionManager.frontCamera.configForVision(0)
            VisionManager.backCamera.configForVision(0)
            VisionManager.frontCamera.setLedMode(LimelightCamera.LedMode.Off)
            VisionManager.backCamera.setLedMode(LimelightCamera.LedMode.Off)
        }
    }
}

val RightStick = HumanControls.t16000m(1) {
    whenButton(Buttons.STICK_BOTTOM) {
        pressed {
            /*
            VisionOdometryUpdater.enable(
                Pose2d(19.0 * 12, 26.0 * 12, Rotation2d.fromDegrees(150.0)),
                VisionManager.frontCamera
            )
            */
            //ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.TestDown)
        }
        released {
            //VisionOdometryUpdater.disable()
            //ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.Disabled)
        }
    }

    whenButton(Buttons.STICK_RIGHT) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbReposition)

            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.Stowed)) {
                //We want to start climbing
                LEDManager.setTrussLedMode(LEDManager.TrussLedMode.Climb)
                LEDManager.setArmLedMode(LEDManager.ArmLedMode.Climb)
                SuperstructureRoutines.ccMaybe(true)
                SuperstructureMotionPlanner.goToClimb()
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.DownL3)
            } else if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.LondonBridgeIsMaybeFallingDown)) {
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.FallL3)
            }

        }
        released {
            //DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbStop)
            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.DownL3)) {
                //We are in the process of climbing, slowly go down
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.SlowFall)
            } else if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.FallL3)) {
                //We have climbed up and pulled ourselves onto the platform, we now want to retract the front legs
                //Additionally, thrust the arm forward to shift our cg
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.LondonBridgeIsMaybeFallingDown)
                SuperstructureRoutines.ccMaybe(true)
                SuperstructureMotionPlanner.climbThrust()
            }
        }
    }

    // Phase 1:
    // Drop the back legs and use the drive-train to get the back wheels on the platform - DownBackL2
    // Drop the front legs and lift the back legs - GoOntoL2
    // Phase 2:
    // Drive onto the platform - ClimbAlign
    // Lift the front legs - LondonBridgeIsMaybeFallingDown

    whenButton(Buttons.STICK_LEFT) {
        pressed {
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbReposition)

            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.Stowed)) {
                //We want to start climbing
                LEDManager.setTrussLedMode(LEDManager.TrussLedMode.Climb)
                LEDManager.setArmLedMode(LEDManager.ArmLedMode.Climb)
                SuperstructureRoutines.ccMaybe(true)
                SuperstructureMotionPlanner.goToClimb()
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.DownBackL2)
            }
        }
        released {
            //DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbStop)
            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.RepositionL2)) {
                // The back of the robot should be on the platform
                // Need to lift the front to drive onto the platform
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.DownFrontL2)
            }
        }
    }
    whenButton(Buttons.STICK_BOTTOM){
        pressed{
            DrivetrainSubsystem.driveMachine.setState(DrivetrainSubsystem.DriveStates.ClimbReposition)

            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.RepositionL2)) {
                // Start level 2 phase 2
                // The robot now has the front on the platform and needs to drive forward
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.GoOntoL2)
            }
        }
        released {
            if (ClimberSubsystem.climberMachine.isInState(ClimberSubsystem.ClimberStates.GoOntoL2)) {
                // The robot should have driven up onto level 2
                ClimberSubsystem.climberMachine.setState(ClimberSubsystem.ClimberStates.LondonBridgeIsMaybeFallingDown)
            }
        }
    }

}


val Gamepad = HumanControls.dualAction(2){
    invertAxis(Axes.LEFT_Y)
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
            SuperstructureRoutines.intake()
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
            Direction.NORTH -> SuperstructureRoutines.goToHigh()
            Direction.WEST -> SuperstructureRoutines.goToMid()
            Direction.SOUTH -> SuperstructureRoutines.goToLow()
            Direction.EAST -> SuperstructureRoutines.goToCargoShip()
        }
    }

    whenButton(Buttons.LEFT_BUMPER) {
        pressed {
            SuperstructureRoutines.switchSides()
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
            SuperstructureMotionPlanner.setToJogMode()
        }
        released {
            SuperstructureMotionPlanner.setToPlanningMode()
        }
    }

    /*
    whenButton(Buttons.START){
        pressed {
            SuperstructureRoutines.ccMaybe(true)
            SuperstructureMotionPlanner.climbThrust()
        }
    }
    */
}