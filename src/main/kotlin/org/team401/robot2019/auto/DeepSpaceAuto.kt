package org.team401.robot2019.auto

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.auto
import org.snakeskin.measure.Inches
import org.team401.robot2019.auto.steps.climber.HomeClimberStep
import org.team401.robot2019.auto.steps.drivetrain.DriveTrajectoryStep
import org.team401.robot2019.auto.steps.drivetrain.EnableVisionStateEstimator
import org.team401.robot2019.auto.steps.drivetrain.OperatorDriveStep
import org.team401.robot2019.auto.steps.drivetrain.WaitForOdometry
import org.team401.robot2019.auto.steps.superstructure.*
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.drivetrain.Trajectories
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

object DeepSpaceAuto: RobotAuto(20L) {
    override fun assembleAuto(): SequentialSteps {
        return auto {
            //Step 1: Arm homing and initial trajectory
            parallel {
                //Arm configuration sequence
                sequential {
                    step(ArmHomeStep()) //The first step in any auto routine is to home the arm.  This can't be interrupted
                    step(SuperstructureGoHomeStep()) //Next, move to the home position
                    step(SuperstructureSwitchToolStep(WristMotionPlanner.Tool.HatchPanelTool)) //Configure for hatch tool
                }

                sequential {
                    step(HomeClimberStep()) //Homes the climber.  This must happen before we can drive.
                    //Drive and prepare to score
                    parallel {
                        step(DriveTrajectoryStep(Trajectories.level1HabToFarRocketLeft, true))
                        /*
                        //Wait for drive to pass an x value, move to scoring position, enable vision
                        sequential {
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 0.0.Inches))
                            step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchHighFront)) //Move to scoring position
                            //step(EnableVisionStateEstimator()) //Enable vision pose updater
                        }
                        */
                    }

                    //step(SuperstructureScoreStep())
                }
            }
        }
    }
}