package org.team401.robot2019.auto

import org.snakeskin.auto.RobotAuto
import org.snakeskin.auto.steps.SequentialSteps
import org.snakeskin.dsl.auto
import org.snakeskin.measure.Inches
import org.snakeskin.measure.Seconds
import org.team401.robot2019.auto.steps.climber.HomeClimberStep
import org.team401.robot2019.auto.steps.drivetrain.*
import org.team401.robot2019.auto.steps.superstructure.*
import org.team401.robot2019.config.ControlParameters
import org.team401.robot2019.control.drivetrain.CriticalPoses
import org.team401.robot2019.control.drivetrain.Trajectories
import org.team401.robot2019.control.superstructure.SuperstructureRoutines
import org.team401.robot2019.control.superstructure.planning.WristMotionPlanner
import org.team401.taxis.trajectory.Trajectory

object DeepSpaceAuto: RobotAuto(20L) {
    const val doesAutoWork = true //oops

    override fun assembleAuto(): SequentialSteps {
        if (!doesAutoWork) {
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

        //val startToRocketTrajectory =

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
                    step(HomeClimberStep()) //Homes the climber.  This must happen before we can drive.
                    //Drive to the near rocket
                    parallel {
                        // Drive to in front of the rocket
                        step(DriveTrajectoryStep(Trajectories.level1HabToLineUpWithRocketLeft, true))

                        sequential {
                            // wait to move the arm until crosses a threshold
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 100.0))
                            // Move the arm to high
                            step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchHighFront))

                        }
                    }

                    // Vision moves to the rocket to score
                    step(VisionAlignStep(SuperstructureRoutines.Side.FRONT, 1.25.Seconds))

                    //Score the hatch
                    step(SuperstructureScoreStep())

                    //Drive from the near rocket to the inbounding station, move to the intake position, enable vision
                    parallel {
                        step(DriveTrajectoryStep(Trajectories.nearRocketLeftToInboundingStationLineUpLeft))

                        sequential {
                            //Wait to move to intake until we cross the x threshold
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.NEGATIVE, 160.0))
                            //Move the arm to the intake position
                            step(SuperstructureIntakeStep(SuperstructureRoutines.Side.BACK))
                        }
                    }

                    //Vision align to the inbounding station
                    step(VisionAlignStep(SuperstructureRoutines.Side.BACK, 1.25.Seconds, .15, 0.9))

                    //Grab the hatch
                    step(SuperstructureStopIntakingStep())

                    //Drive from the inbounding station to the rocket, move to scoring position, enable vision
                    parallel {
                        step(DriveTrajectoryStep(Trajectories.inboundingStationLeftToNearRocketLineUpLeft))

                        sequential {
                            step(WaitForOdometry(WaitForOdometry.Axis.X, WaitForOdometry.Direction.POSITIVE, 110.0))
                            step(SuperstructureMoveStep(ControlParameters.SuperstructurePositions.rocketHatchMidFront))
                        }
                    }

                    //Vision align to the inbounding station
                    step(VisionAlignStep(SuperstructureRoutines.Side.FRONT, 1.5.Seconds, .2, .9))

                    //Score the hatch
                    step(SuperstructureScoreStep())

                    step(OpenLoopReverseStep(0.25, 1.0.Seconds))
                }
            }
            step(OperatorDriveStep())
        }
    }
}