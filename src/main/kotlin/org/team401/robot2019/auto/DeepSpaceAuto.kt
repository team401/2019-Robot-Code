package org.team401.robot2019.auto

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.auto
import org.snakeskin.measure.Seconds
import org.team401.robot2019.auto.steps.climber.HomeClimberStep
import org.team401.robot2019.auto.steps.drivetrain.*
import org.team401.robot2019.auto.steps.superstructure.*
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.drivetrain.Trajectories
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner

object DeepSpaceAuto: RobotAuto(20L) {
    private val autoChooser = AutoMode.toSendableChooser()

    fun publish() {
        SmartDashboard.putData("Auto Mode", autoChooser)
        //DriverStationDisplay.autoSelctor.setValue(AutoMode.toSendableChooser())
    }

    override fun assembleAuto(): SequentialSteps {
        val selectedAuto = AutoMode.Manual//autoChooser.selected//DriverStationDisplay.autoSelctor.value.value as AutoMode
        println("Selected auto: ${selectedAuto}")

        if (selectedAuto == AutoMode.Manual) {
            return auto {
                parallel {
                    step(PrepareDriverVisionStep())
                    step(OperatorDriveStep())
                    sequential {
                        step(ArmHomeStep()) //The first step in any auto routine is to home the arm.  This can't be interrupted
                        step(SuperstructureGoHomeStep()) //Next, move to the home position
                        step(SuperstructureSwitchToolStep(WristMotionPlanner.Tool.HatchPanelTool)) //Configure for hatch tool
                    }
                    step(HomeClimberStep()) //Homes the climber.
                }
            }
        }

        val rocketToLoadingTrajectory = Trajectories.rocketLeftToHatchIntakeLeft
        val loadingToRocketTrajectory = Trajectories.hatchIntakeLeftToRocketLeft


        return auto {
            //Step 1: Arm homing and initial trajectory
            parallel {
                //Prepare cameras
                //step(PrepareVisionStep())

                //Arm configuration sequence
                sequential {
                    step(ArmHomeStep()) //The first step in any auto routine is to home the arm.  This can't be interrupted
                    step(SuperstructureGoHomeStep()) //Next, move to the home position
                    step(SuperstructureSwitchToolStep(WristMotionPlanner.Tool.HatchPanelTool)) //Configure for hatch tool
                }

                sequential {
                    //Drive to the near rocket
                    parallel {
                        step(HomeClimberStep()) //Homes the climber.  This must happen before we can drive.
                        step(OperatorDriveStep())
                        step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchBottomFront))
                    }

                    //Wait for scoring the hatch
                    step(WaitForScoreButtonStep())

                    //Start auto routine

                    //Drive to loading and move arm and intake
                    parallel {
                        step(DriveTrajectoryStep(rocketToLoadingTrajectory, true))
                        sequential {
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.NEGATIVE, 120.0))
                            step(SuperstructureIntakeHatchStep())
                            step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.hatchIntakeBack))
                        }
                    }

                    //Wait to get the hatch
                    step(WaitForHatchAcquiredStep())

                    //Drive back to the rocket
                    parallel {
                        step(DriveTrajectoryStep(loadingToRocketTrajectory, false))
                        sequential {
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 120.0))
                            step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchMidFront))
                        }
                    }

                    //Score the hatch
                    step(SuperstructureScoreStep())
                    step(OpenLoopReverseStep(.25, 1.0.Seconds))
                    step(SuperstructureStopScoringStep())
                }
            }
        }
    }
}