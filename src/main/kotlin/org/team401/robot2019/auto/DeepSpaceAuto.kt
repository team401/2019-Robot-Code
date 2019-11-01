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
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.robot2019.subsystems.DrivetrainSubsystem
import org.team401.robot2019.subsystems.WristSubsystem

object DeepSpaceAuto: RobotAuto(20L) {
    private val autoChooser = AutoMode.toSendableChooser()

    fun publish() {
        SmartDashboard.putData("Auto Mode", autoChooser)
        //DriverStationDisplay.autoSelctor.setValue(AutoMode.toSendableChooser())
    }

    override fun assembleAuto(): SequentialSteps {
        val selectedAuto = autoChooser.selected
        println("Selected auto: $selectedAuto")

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

        val habToRocketTrajectory = if (selectedAuto == AutoMode.TwoHatchLeft) Trajectories.habToRocketLeft else Trajectories.habToRocketRight
        val rocketToLoadingTrajectory = if (selectedAuto == AutoMode.TwoHatchLeft) Trajectories.rocketLeftToStation else Trajectories.rocketRightToStation
        val loadingToRocketTrajectory = if (selectedAuto == AutoMode.TwoHatchLeft) Trajectories.stationToRocketLeft else Trajectories.stationToRocketRight

        return auto {
            parallel {
                step(DriveTrajectoryStep(habToRocketTrajectory, true, DrivetrainSubsystem.VisionContinuanceMode.ContinueRocketFront))
                step(HomeClimberStep()) //Homes the climber.  This must happen before we can drive.
                sequential {
                    step(SuperstructureSwitchToolStep(WristMotionPlanner.Tool.HatchPanelTool)) //Configure for hatch tool
                    step { WristSubsystem.wheelsMachine.setState(WristSubsystem.WristWheelsStates.Holding) }
                    step(ArmHomeStep()) //The first step in any auto routine is to home the arm.  This can't be interrupted
                    step(SuperstructureGoHomeStep()) //Next, move to the home position

                    step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 90.0))
                    step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchHighFront))
                }
            }
            step(SuperstructureScoreStep())

            parallel {
                step(DriveTrajectoryStep(rocketToLoadingTrajectory, true, DrivetrainSubsystem.VisionContinuanceMode.ContinueHatchBack))
                sequential {
                    step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.NEGATIVE, 13.0 * 12.0))
                    step(SuperstructureIntakeHatchStep())
                    step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.hatchIntakeBack))
                }
            }

            parallel {
                step(DriveTrajectoryStep(loadingToRocketTrajectory, false, DrivetrainSubsystem.VisionContinuanceMode.ContinueRocketFront))
                sequential {
                    step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 6.0 * 12.0))
                    step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchMidFront))
                }
            }

            step(SuperstructureScoreStep())
            step(OpenLoopReverseStep(.25, 1.0.Seconds))
            step(SuperstructureStopScoringStep())
        }
    }
}