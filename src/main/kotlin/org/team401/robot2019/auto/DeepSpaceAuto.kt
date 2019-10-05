package org.team401.robot2019.auto

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.auto
import org.snakeskin.measure.Seconds
import org.team401.robot2019.DriverStationDisplay
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

        val startToRocketTrajectory = when (selectedAuto.side) {
            AutoSide.Left -> Trajectories.level1HabToLineUpWithRocketLeft
            AutoSide.Right -> Trajectories.level1HabToLineUpWithRocketRight
        }

        val rocketToInboundingTrajectory = when (selectedAuto.side) {
            AutoSide.Left -> Trajectories.nearRocketLeftToInboundingStationLineUpLeft
            AutoSide.Right -> Trajectories.nearRocketRightToInboundingStationLineUpRight
        }

        val inboundingToRocketTrajectory = when (selectedAuto.side) {
            AutoSide.Left -> Trajectories.inboundingStationLeftToNearRocketLineUpLeft
            AutoSide.Right -> Trajectories.inboundingStationRightToNearRocketLineUpRight
        }

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
                        // Drive to in front of the rocket
                        step(DriveTrajectoryStep(startToRocketTrajectory, true))

                        sequential {
                            // wait to move the arm until crosses a threshold
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 100.0))
                            // Move the arm to high
                            step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchHighFront))

                        }
                    }

                    // Vision moves to the rocket to score
                    step(VisionAlignStep(SuperstructureRoutines.Side.FRONT, 1.4.Seconds))

                    //Score the hatch
                    step(SuperstructureScoreStep())

                    //Drive from the near rocket to the inbounding station, move to the intake position, enable vision
                    parallel {
                        step(DriveTrajectoryStep(rocketToInboundingTrajectory))

                        sequential {
                            //Wait to move to intake until we cross the x threshold
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.NEGATIVE, 160.0))
                            //Move the arm to the intake position
                            step(SuperstructureIntakeStep(SuperstructureRoutines.Side.BACK))
                        }
                    }

                    //Vision align to the inbounding station
                    step(VisionAlignStep(SuperstructureRoutines.Side.BACK, 1.8.Seconds, .15, 0.9))

                    //Grab the hatch
                    step(SuperstructureStopIntakingStep())

                    //Drive from the inbounding station to the rocket, move to scoring position, enable vision
                    parallel {
                        step(DriveTrajectoryStep(inboundingToRocketTrajectory))

                        sequential {
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 110.0))
                            step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchMidFront))
                        }
                    }

                    //Vision align to the inbounding station
                    step(VisionAlignStep(SuperstructureRoutines.Side.FRONT, 1.9.Seconds, .15, .9))

                    //Score the hatch
                    step(SuperstructureScoreStep())

                    step(OpenLoopReverseStep(0.25, 1.0.Seconds))
                }
            }
            step(OperatorDriveStep())
        }
    }
}